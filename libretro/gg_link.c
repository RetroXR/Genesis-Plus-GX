/*
 * Gear-to-Gear cable carried by the frontend's link bus.
 *
 * The Game Gear's EXT connector is seven general-purpose pins (PC0-PC6), two of
 * which double as a UART. The Gear-to-Gear cable crosses them, so that what one
 * machine drives the other reads:
 *
 *    PC0 <-> PC2    PC1 <-> PC3    PC4 (TXD) <-> PC5 (RXD)    PC6 <-> PC6
 *
 * Nearly every link game uses the UART. Port $05 bits 4-5 switch it on, $03
 * takes a byte to send and $04 holds the one received; bit 0 of $05 says the
 * send buffer is full, bit 1 that a byte has arrived, bit 2 that the other
 * machine is not there, and bit 3 asks for an NMI whenever a byte arrives or the
 * far end is missing -- which is how games find out, since they poll nothing.
 * The parallel mode (Jon's Squinky Tennis) is polled and has no handshake; it is
 * carried by publishing the pins a machine drives whenever they change.
 *
 * WHAT CROSSES. Three messages. GL_PINS says which EXT pins a machine drives and
 * to what; GL_BYTE is a byte it has sent; GL_ACK says its receive buffer was
 * read, which is what empties the sender's send buffer. That last is BizHawk's
 * reading of the hardware, whose GGHawkLink runs the commercial link games: a
 * byte's send-buffer-full bit stays up until the far end has taken the byte.
 *
 * WHAT DOES NOT. A PC6 edge raising an NMI in parallel mode. A byte lands ten
 * bit times after it was written, at the rate $05 selects.
 *
 * Time is Genesis Plus GX's own master clock (15 per Z80 cycle), counted from
 * attach, so nothing needs converting and a frame is 262 * 3420 ticks.
 */

#include <string.h>

#include "shared.h"
#include "gg_link.h"
#include <libretro.h>

extern retro_log_printf_t log_cb;

/* At WARN, because RetroXR drops everything below it and these are the lines
 * that say whether a cable is doing anything. */
#define GL_LOG(...) do { if (log_cb) log_cb(RETRO_LOG_WARN, __VA_ARGS__); } while (0)

#define GL_PROTOCOL "gg-ext-1"

/* Rendezvous every four scanlines and promise a horizon of the same.
 *
 * A byte at the fastest rate the UART offers, 4800 bps, is ten bit times: 2 ms,
 * or about 33 lines. Four lines is an eighth of that, so a byte arriving a
 * horizon late is still well inside the time it would have taken on the wire,
 * and it costs 66 rendezvous a frame, the same order as gambatte pays. */
#define GL_LINES 4
#define GL_GRAIN_SERIAL ((uint64_t)(GL_LINES * MCYCLES_PER_LINE))

/* ...and every line while a machine drives any EXT pin itself. Mortal Kombat
 * and Mortal Kombat II bit-bang the parallel pins -- PC0 a clock the far end
 * answers -- and give up with LINK ERROR when an answer takes four lines to come
 * back. One line is the floor: the core only rendezvouses between lines, and a
 * horizon shorter than the grain deadlocks two machines that each wait for the
 * other to be a grain ahead. */
#define GL_GRAIN_PINS ((uint64_t)MCYCLES_PER_LINE)

#define GL_GRAIN grain()
#define GL_HORIZON GL_GRAIN

/* Rendezvous after the cable moves during which a machine keeps saying what its
 * pins hold. See gambatte's LinkSerial: a message sent while the peer is off its
 * timeline is dropped, and parallel pins may not change again for minutes. */
#define GL_REANNOUNCE_GRAINS 8

enum { GL_PINS = 1, GL_BYTE, GL_ACK };

#define GL_MSG_SIZE 8
#define GL_PENDING_MAX 32

/* $05 */
#define SC_TXFL 0x01
#define SC_RXRD 0x02
#define SC_FRER 0x04
#define SC_INT  0x08
#define SC_ON   0x30

typedef struct
{
  uint64_t tick;
  uint8 type, data, mask;
} gl_pending_t;

static const struct retro_link_interface *link_;
static struct retro_link_interface link_copy_;
static retro_link_port_t *handle_;
static int attached_;
static unsigned peers_;

static uint64_t base_;      /* link tick at Z80 cycle 0 of this frame */
static uint64_t last_;      /* never let the published position go back */
static uint64_t grant_;
static int anchored_;
static uint64_t reannounce_until_;

static uint8 peer_out_, peer_driven_;
static uint8 last_out_, last_driven_;
static int pins_published_;
static int nmi_due_;

static gl_pending_t pending_[GL_PENDING_MAX];
static unsigned pending_count_;

/* Which of the peer's pins each of this machine's pins is wired to. */
static const uint8 cross_[7] = { 2, 3, 0, 1, 5, 4, 6 };

static uint64_t grain(void)
{
  return (~io_reg[2] & 0x7F) ? GL_GRAIN_PINS : GL_GRAIN_SERIAL;
}

/* The promise, which may never be retracted: shrinking the horizon when a game
 * starts driving pins must not publish a safe tick below the last one. */
static uint64_t safe_;

static uint64_t safe_at(uint64_t now)
{
  uint64_t t = now + GL_HORIZON;
  if (t < safe_)
    t = safe_;
  safe_ = t;
  return t;
}

static uint64_t now_at(unsigned int cycles)
{
  uint64_t t = base_ + cycles;
  if (t < last_)
    t = last_;
  last_ = t;
  return t;
}

static void anchor(uint64_t now)
{
  if (!anchored_)
  {
    grant_ = link_->advance(handle_, now, safe_at(now), now, 0);
    anchored_ = 1;
  }
}

static void wire_at(uint64_t now, uint64_t lands, uint8 type, uint8 data, uint8 mask)
{
  uint8 msg[GL_MSG_SIZE];

  anchor(now);
  memset(msg, 0, sizeof(msg));
  msg[0] = type;
  msg[2] = data;
  msg[3] = mask;

  /* Stamped a horizon out: the promise this machine published, and the one tick
   * the peer is guaranteed not to have run past. */
  link_->send(handle_, lands, RETRO_LINK_BROADCAST, msg, sizeof(msg));
}

static void wire(uint64_t now, uint8 type, uint8 data, uint8 mask)
{
  wire_at(now, safe_at(now), type, data, mask);
}

/* How long a byte takes down the wire: a start bit, eight data bits and a stop
 * bit at the rate $05 bits 6-7 select (4800, 2400, 1200 or 300 bps).
 *
 * This is not decoration. Faceball 2000 answers every byte from inside the NMI
 * that received it, so the two machines play ping-pong for as long as the link
 * is up; delivered a horizon after being written -- an eighth of a real byte
 * time -- the rally ran eight times too fast, and the machine that booted first
 * spent every cycle in its NMI and never drew another frame. */
static uint64_t byte_ticks(void)
{
  static const unsigned bps[4] = { 4800, 2400, 1200, 300 };
  uint64_t t = (uint64_t)system_clock * 10 / bps[io_reg[5] >> 6];
  return t > GL_HORIZON ? t : GL_HORIZON;
}

static int serial_on(void)
{
  return (io_reg[5] & SC_ON) == SC_ON;
}

static void publish_pins(uint64_t now)
{
  uint8 driven = (uint8)(~io_reg[2] & 0x7F);
  uint8 out = io_reg[1] & driven;

  if (peers_ < 2)
    return;
  if (pins_published_ && out == last_out_ && driven == last_driven_)
    return;

  last_out_ = out;
  last_driven_ = driven;
  pins_published_ = 1;
  wire(now, GL_PINS, out, driven);
}

static void apply(const gl_pending_t *msg)
{
  switch (msg->type)
  {
    case GL_PINS:
      peer_out_ = msg->data;
      peer_driven_ = msg->mask;
      break;

    case GL_BYTE:
      /* A receiver with its UART off hears nothing, as on the hardware. */
      if (serial_on())
      {
        io_reg[4] = msg->data;
        io_reg[5] |= SC_RXRD;
        if (io_reg[5] & SC_INT)
          nmi_due_ = 1;
      }
      break;

    case GL_ACK:
      io_reg[5] &= ~SC_TXFL;
      break;

    default:
      break;
  }
}

static void apply_due(uint64_t now)
{
  unsigned i = 0;

  while (i < pending_count_ && pending_[i].tick <= now)
    apply(&pending_[i++]);

  if (i)
  {
    memmove(&pending_[0], &pending_[i], sizeof(gl_pending_t) * (pending_count_ - i));
    pending_count_ -= i;
  }
}

static void pump(void)
{
  uint8 buf[GL_MSG_SIZE];
  uint64_t tick;
  unsigned from;
  size_t len = sizeof(buf);

  while (link_->recv(handle_, &tick, &from, buf, &len))
  {
    if (len == GL_MSG_SIZE)
    {
      if (pending_count_ < GL_PENDING_MAX)
      {
        /* Kept in tick order: a byte is stamped a byte time out, so a pin
         * change sent after it can land before it. */
        unsigned at = pending_count_++;
        gl_pending_t *p;
        while (at > 0 && pending_[at - 1].tick > tick)
        {
          pending_[at] = pending_[at - 1];
          at--;
        }
        p = &pending_[at];
        p->tick = tick;
        p->type = buf[0];
        p->data = buf[2];
        p->mask = buf[3];
      }
      else
      {
        /* Never acted on early to make room: when it would land would then be
         * decided by how many messages had arrived, which is wall-clock luck. */
        GL_LOG("Gear-to-Gear: inbox full, dropping a message\n");
      }
    }
    len = sizeof(buf);
  }
}

static void refresh_peers(uint64_t now)
{
  unsigned was = peers_;
  unsigned count = 0;
  int id = link_->peers(handle_, &count);

  if (id < 0)
    peers_ = 0;
  else if (count > 2)
  {
    /* A Gear-to-Gear cable joins two machines and nothing more. */
    GL_LOG("Gear-to-Gear: %u machines on one wire; the cable carries 2\n", count);
    peers_ = 0;
  }
  else
    peers_ = count;

  if (peers_ != was)
  {
    /* Forget the far end and say everything again. Anything in flight belonged
     * to the old cable, and a byte that was in it will never be taken, so the
     * send buffer it filled is emptied rather than left full for ever. */
    peer_out_ = 0;
    peer_driven_ = 0;
    pins_published_ = 0;
    anchored_ = 0;
    io_reg[5] &= ~SC_TXFL;

    /* Only a cable coming OUT empties the inbox. When one goes in, the two
     * machines notice at their own next rendezvous, so the first to notice may
     * already have sent a byte the second is holding -- throw that away and the
     * sender's buffer stays full for ever, since nobody will ever take it. */
    if (peers_ < 2)
      pending_count_ = 0;
    reannounce_until_ = now + GL_REANNOUNCE_GRAINS * GL_GRAIN;
    GL_LOG("Gear-to-Gear: %u machine(s) on the wire, this one is %d\n", peers_, id);
  }
}

/* ---- hooks the core calls -------------------------------------------------- */

static void line_hook(unsigned int cycles)
{
  uint64_t now = now_at(cycles);
  uint32_t wake = RETRO_LINK_WAKE_NONE;

  if (now + MCYCLES_PER_LINE > grant_)
  {
    refresh_peers(now);

    /* Publish before reading, as gambatte does: a peer parked on this machine's
     * horizon may be sitting on the very message this one is about to want.
     * Asking for a whole grain always covers the line about to run, so the Z80
     * never has to be stopped part way through one. */
    grant_ = link_->advance(handle_, now, safe_at(now), now + GL_GRAIN, &wake);
    anchored_ = 1;

    /* Nothing bounds this machine, but it still has to come back and look: a
     * lead plugged in later is only noticed at a rendezvous. */
    if (grant_ == RETRO_LINK_UNBOUNDED || grant_ < now + GL_GRAIN)
      grant_ = now + GL_GRAIN;

    if (peers_ >= 2 && now < reannounce_until_)
      pins_published_ = 0;
  }

  pump();
  apply_due(now);
  publish_pins(now);

  if (nmi_due_)
  {
    nmi_due_ = 0;
    z80_set_nmi_line(ASSERT_LINE);
    z80_set_nmi_line(CLEAR_LINE);
  }
}

static void frame_hook(unsigned int cycles)
{
  /* Bit 2 is NOT raised with nothing on the wire. It is a framing error: SMS
   * Power measured it "raised repeatedly" with the cable in and the far Game Gear
   * switched OFF, which pulls the receive line low. With no cable the line idles
   * high and nothing fires, which is what the stock core gives -- and raising it
   * anyway (BizHawk's reading) hangs Streets of Rage at boot, whose NMI handler
   * never expected a storm with no lead in. The bus cannot tell "no cable" from
   * "far end off", so both read as no cable. */
  io_reg[5] &= ~SC_FRER;

  now_at(cycles);
  base_ += cycles;
}

static void write_hook(unsigned int offset, unsigned int data)
{
  uint64_t now = now_at(Z80.cycles);
  (void)data;

  switch (offset)
  {
    case 1:
    case 2:
      publish_pins(now);
      break;

    case 3:
      if (serial_on() && peers_ >= 2)
      {
        io_reg[5] |= SC_TXFL;
      {
        uint64_t lands = now + byte_ticks();
        uint64_t safe = safe_at(now);
        wire_at(now, lands > safe ? lands : safe, GL_BYTE, io_reg[3], 0);
      }
      }
      break;

    default:
      break;
  }
}

static unsigned int read_hook(unsigned int offset, unsigned int value)
{
  uint64_t now = now_at(Z80.cycles);
  uint8 in = 0;
  int p;

  pump();
  apply_due(now);

  switch (offset)
  {
    case 1:
      /* Inputs read the far end through the crossed wiring; a pin nobody
       * drives floats high, which is what the port read already gave. */
      for (p = 0; p < 7; p++)
      {
        uint8 q = cross_[p];
        if (!(peer_driven_ & (1 << q)) || (peer_out_ & (1 << q)))
          in |= (uint8)(1 << p);
      }
      /* Bit 7 is no pin at all and reads high. Pete Sampras Tennis finds its
       * partner by waiting for $01 to read exactly CF (and the answer F0), bit 7
       * included; the core returns the latch there, 0 after a reset, so the two
       * could never match. Only while cabled, so a machine with no lead in reads
       * exactly what the stock core gives. */
      value = (value & ~(io_reg[2] & 0x7F)) | (in & io_reg[2] & 0x7F);
      return peers_ >= 2 ? (value | 0x80) : value;

    case 4:
      if (serial_on() && (io_reg[5] & SC_RXRD))
      {
        io_reg[5] &= ~SC_RXRD;
        if (peers_ >= 2)
          wire(now, GL_ACK, 0, 0);
      }
      return io_reg[4];

    case 5:
      return io_reg[5];

    default:
      return value;
  }
}

/* ---- libretro side ---------------------------------------------------------- */

void gg_link_init(retro_environment_t environ_cb)
{
  memset(&link_copy_, 0, sizeof(link_copy_));
  link_ = NULL;
  /* The API proposed upstream in libretro/RetroArch#19454, vendored into this
   * core's libretro.h as that pull request adds it. */
  if (environ_cb(RETRO_ENVIRONMENT_GET_LINK_INTERFACE, &link_copy_))
    link_ = &link_copy_;
}

void gg_link_start(void)
{
  if (attached_ || !link_)
    return;
  if (system_hw != SYSTEM_GG && system_hw != SYSTEM_GGMS)
    return;

  handle_ = link_->attach(0, GL_PROTOCOL, (uint64_t)system_clock);
  if (!handle_)
  {
    GL_LOG("Gear-to-Gear: the frontend refused the EXT port\n");
    return;
  }

  attached_ = 1;
  base_ = last_ = grant_ = safe_ = 0;
  anchored_ = 0;
  peers_ = 0;
  peer_out_ = peer_driven_ = 0;
  pins_published_ = 0;
  nmi_due_ = 0;
  pending_count_ = 0;
  refresh_peers(0);

  gg_link_line = line_hook;
  gg_link_frame = frame_hook;
  gg_link_write = write_hook;
  gg_link_read = read_hook;
  GL_LOG("Gear-to-Gear: attached, %u machine(s) on the wire\n", peers_);
}

void gg_link_stop(void)
{
  gg_link_line = NULL;
  gg_link_frame = NULL;
  gg_link_write = NULL;
  gg_link_read = NULL;

  if (!attached_)
    return;
  link_->detach(handle_);
  handle_ = NULL;
  attached_ = 0;
  peers_ = 0;
}

void gg_link_reset(void)
{
  /* The master clock the core counts from restarts with the machine; the bus
   * must never see this one go backwards, so base_ carries on. */
  pending_count_ = 0;
  pins_published_ = 0;
  anchored_ = 0;
  nmi_due_ = 0;
}
