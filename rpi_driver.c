/*
 * Art-Net -> WS2812 bridge (RPi SMI/DMA) with multi-universe
 * - 170 LEDs per universe (510 data bytes)
 * - Configurable starting universe
 * - N_LAMPS total RGB pixels (3 DMX channels per lamp)
 */

#include <stdio.h>
#include <stdbool.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <pthread.h>
#include <artnet/artnet.h>
#include <artnet/packets.h>
#include <sys/time.h>

/* ======== PIXLED includes (unchanged) ======== */
#include "rpi_dma_utils.h"
#include "rpi_smi_defs.h"

/* ----------------- USER CONFIG ----------------- */
#define N_LAMPS             900           /* total RGB pixels */
#define START_UNIVERSE_DEF  0              /* default starting universe */

#define LED_NCHANS          8
#define LED_D0_PIN          8
#define DMA_CHAN            10

#if PHYS_REG_BASE==PI_4_REG_BASE
#define SMI_TIMING          10, 15, 30, 15
#else
#define SMI_TIMING          10, 10, 20, 10
#endif

#define LED_NBITS           24
#define LED_PREBITS         10
#define LED_POSTBITS        80
#define BIT_NPULSES         3
#define REQUEST_THRESH      2


const int interval_ms = 75;

/* ======== Derived constants ======== */
#if LED_NCHANS > 8
#define TXDATA_T            uint16_t
#else
#define TXDATA_T            uint8_t
#endif
#define LED_DLEN            (LED_NBITS * BIT_NPULSES)
#define LED_TX_OSET(n)      (LED_PREBITS + (LED_DLEN * (n)))
#define TX_BUFF_LEN(n)      (LED_TX_OSET(n) + LED_POSTBITS)
#define TX_BUFF_SIZE(n)     (TX_BUFF_LEN(n) * sizeof(TXDATA_T))
#define CHAN_MAXLEDS        N_LAMPS
#define VC_MEM_SIZE         (PAGE_SIZE + TX_BUFF_SIZE(CHAN_MAXLEDS))

/* DMX / Art-Net */
#define DMX_SLOTS           512
#define DMX_DATA_LEN        510              /* usable DMX per universe */
#define LEDS_PER_UNI        170              /* 170 * 3 = 510 */
#define MAX_UNIS            ARTNET_MAX_PORTS * 2
#define VERBOSE             0 /* 0 means VERBOSE*/
extern MEM_MAP gpio_regs, dma_regs;
MEM_MAP vc_mem, clk_regs, smi_regs;

volatile SMI_CS_REG  *smi_cs;
volatile SMI_L_REG   *smi_l;
volatile SMI_A_REG   *smi_a;
volatile SMI_D_REG   *smi_d;
volatile SMI_DMC_REG *smi_dmc;
volatile SMI_DSR_REG *smi_dsr;
volatile SMI_DSW_REG *smi_dsw;
volatile SMI_DCS_REG *smi_dcs;
volatile SMI_DCA_REG *smi_dca;
volatile SMI_DCD_REG *smi_dcd;

TXDATA_T *txdata;
TXDATA_T tx_buffer[TX_BUFF_LEN(CHAN_MAXLEDS)];
int      rgb_data[CHAN_MAXLEDS][LED_NCHANS];
bool	 already_recv_uni[MAX_UNIS];
bool	data_changed;

static int start_universe = START_UNIVERSE_DEF;
int needed_universes = 1;
static pthread_mutex_t dmx_mtx = PTHREAD_MUTEX_INITIALIZER;

/* Static DMX buffers (no calloc) */
static uint8_t dmx_uni[MAX_UNIS][DMX_DATA_LEN];
static int     dmx_len[MAX_UNIS];  /* received length per universe [0..510] */


static void rgb_txdata(int *rgbs, TXDATA_T *txd);
static void map_devices(void);
static void init_smi(int width, int ns, int setup, int hold, int strobe);
static void setup_smi_dma(MEM_MAP *mp, int nsamp);
static void start_smi(MEM_MAP *mp);
void terminate(int sig);
void swap_bytes(void *data, int len);

static inline int min_i(int a, int b) { return a < b ? a : b; }

static inline uint8_t dmx_get_abs(int idx)
{
    if (idx < 0) return 0;
    int u = idx / DMX_DATA_LEN;
    int o = idx % DMX_DATA_LEN;
    if (u < 0 || u >= needed_universes) return 0;
    if (o >= dmx_len[u]) return 0;
    return dmx_uni[u][o];
}

static void update_leds_from_dmx(void)
{
 /*   if(!data_changed) return;
    data_changed = 0;*/
    pthread_mutex_lock(&dmx_mtx);

    const int max_leds = N_LAMPS;
    int leds_red = 0;
    for (int n = 0; n < max_leds; n++) {
        int base = n * 3;
        int r = dmx_get_abs(base + 0);
        int g = dmx_get_abs(base + 1);
        int b = dmx_get_abs(base + 2);
	if (r == 255) leds_red++;
        rgb_data[n][1] = (r << 16) | (g << 8) | b; /* RGB (rgb_txdata handles GRB on wire) */
    }
    printf("reds %d\n", leds_red);
    pthread_mutex_unlock(&dmx_mtx);

    for (int n = 0; n < N_LAMPS; n++)
        rgb_txdata(rgb_data[n], &tx_buffer[LED_TX_OSET(n)]);
    
    swap_bytes(tx_buffer, TX_BUFF_SIZE(N_LAMPS));
    memcpy(txdata, tx_buffer, TX_BUFF_SIZE(N_LAMPS));
    start_smi(&vc_mem);
    usleep(10);
    while (dma_active(DMA_CHAN))
	usleep(10);
}


long long now_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}


static int dmx_handler(artnet_node n, int port, void *userdata)
{
    (void)userdata;
    
    int uni_number = artnet_get_universe_addr(n, port, ARTNET_OUTPUT_PORT);
    int global_u   = uni_number - start_universe;
    //printf("recved uni %d %d\n", uni_number, global_u);
    
    if (global_u < 0 || global_u >= needed_universes)
        return 0;  /* not one we're mapping */

    int len = 0;
    uint8_t *data = artnet_read_dmx(n, port, &len);
    if (!data || len <= 0) return 0;

    if (len > DMX_DATA_LEN) len = DMX_DATA_LEN;
    if (needed_universes == global_u) len = (LED_NCHANS%170)*3;

    global_u = global_u-1;
    pthread_mutex_lock(&dmx_mtx);
    
    //if (memcmp(dmx_uni[global_u], data, (size_t)len)) {
    	memcpy(dmx_uni[global_u], data, (size_t)len);
    	dmx_len[global_u] = len;
    //	data_changed = 1;
   // }	
    pthread_mutex_unlock(&dmx_mtx);

    /*
    if (already_recv_uni[global_u]){
	update_leds_from_dmx();
	printf("double set uni %d, updating\n", global_u);
	memset(already_recv_uni, 0, MAX_UNIS);
	return 0;
    }
    already_recv_uni[global_u] = true;
    int sum = 0;
    for (int i = 0; i < MAX_UNIS; i++) {
        sum += already_recv_uni[i];  // true adds 1, false adds 0
    }
    printf("sum %d, needed_uni %d\n", sum, needed_universes);
    if ((needed_universes)-1>sum) return 0;
    memset(already_recv_uni, 0, MAX_UNIS);
    printf("all uni set once, updating\n");

    update_leds_from_dmx();
    */
    return 0;
}

static void setup_artnet_multi_nodes(void)
{
    artnet_node node1 = artnet_new((char*)NULL, VERBOSE);
    artnet_node node2 = artnet_new((char*)"10.10.10.9", VERBOSE);   /* may be NULL to auto-bind */

    if (!node1 || !node2) {
        fprintf(stderr, "artnet_new failed (node1=%p node2=%p)\n", (void*)node1, (void*)node2);
        exit(1);
    }

    artnet_join(node1, node2);

    artnet_set_short_name(node1, "LEDRING360ROT Node 0");
    artnet_set_long_name(node1,  "LEDRING");
    artnet_set_node_type(node1, ARTNET_NODE);
    artnet_set_dmx_handler(node1, dmx_handler, NULL);

    artnet_set_short_name(node2, "LEDRING360ROT Node 0");
    artnet_set_long_name(node2,  "LEDRING");
    artnet_set_node_type(node2, ARTNET_NODE);
    artnet_set_dmx_handler(node2, dmx_handler, NULL);

    int total_ports = needed_universes;             
    if (total_ports > MAX_UNIS) total_ports = MAX_UNIS;

  for(int i=0; i<=total_ports; i++) {
    if(i < 5) {
      artnet_set_port_addr(node1, i%4, ARTNET_OUTPUT_PORT, i);
      artnet_set_subnet_addr(node1, 0);
    } else {
      artnet_set_subnet_addr(node2, 0);
      artnet_set_port_addr(node2, i%4, ARTNET_OUTPUT_PORT, i);
    }
    }

    if (artnet_start(node1) != ARTNET_EOK) {
        fprintf(stderr, "artnet_start node1 failed: %s\n", artnet_strerror());
        exit(1);
    }
    if (artnet_start(node2) != ARTNET_EOK) {
        fprintf(stderr, "artnet_start node2 failed: %s\n", artnet_strerror());
        exit(1);
    }

    /* Shared socket from node1 */
    int sd = artnet_get_sd(node1);


    printf("Serving %d universe(s) starting at %d via 2 nodes (max 8)\n",
           total_ports, start_universe);

    long long last = now_ms();
    /* Poll with select() and feed node1 (joined socket) */
    while (1) {
        fd_set rset;
        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
        FD_ZERO(&rset);
        FD_SET(sd, &rset);

        int rv = select(sd+1, &rset, NULL, NULL, &tv);
        if (rv <= 0) continue;  /* timeout or EINTR */

        if (FD_ISSET(sd, &rset)) {
            /* artnet_read on node1 is enough because of artnet_join */
            artnet_read(node1, 0);
        }
        long long t = now_ms();
        if (t - last >= interval_ms) {
            // 🔹 this part happens every interval_ms
            printf("Triggered at %lld ms\n", t);
	    update_leds_from_dmx();
            last = t; // reset timer
        }
    }
}

int main(int argc, char *argv[])
{
    /* Optional CLI: -u <start_universe> */
    for (int i = 1; i + 1 < argc; i++) {
        if ((strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--universe") == 0)) {
            start_universe = atoi(argv[i+1]);
        }
    }

    /* Work out how many universes we need for N_LAMPS */
    int needed_bytes = N_LAMPS * 3;
    needed_universes = (needed_bytes + DMX_DATA_LEN - 1) / DMX_DATA_LEN;
    if (needed_universes < 1) needed_universes = 1;
    if (needed_universes > MAX_UNIS) {
        fprintf(stderr, "Error: Need %d universes but only %d ports supported.\n",
                needed_universes, MAX_UNIS);
        return 1;
    }

    /* ---- Pixled init ---- */
    signal(SIGINT, terminate);
    map_devices();
    init_smi(LED_NCHANS>8 ? SMI_16_BITS : SMI_8_BITS, SMI_TIMING);
    map_uncached_mem(&vc_mem, VC_MEM_SIZE);
    setup_smi_dma(&vc_mem, TX_BUFF_LEN(N_LAMPS));
    
    memset(dmx_uni, 0, sizeof(dmx_uni));
    
    memset(dmx_len, 0, sizeof(dmx_len));
    memset(rgb_data, 0, sizeof(rgb_data));

    update_leds_from_dmx(); /* send “all off” once */
    

    /* Multi-node Art-Net */
    setup_artnet_multi_nodes();


    return 0;
}

/* ================== Pixled helpers (unchanged logic) ================== */
static void map_devices(void)
{
    map_periph(&gpio_regs, (void *)GPIO_BASE, PAGE_SIZE);
    map_periph(&dma_regs,  (void *)DMA_BASE,  PAGE_SIZE);
    map_periph(&clk_regs,  (void *)CLK_BASE,  PAGE_SIZE);
    map_periph(&smi_regs,  (void *)SMI_BASE,  PAGE_SIZE);
}

static void init_smi(int width, int ns, int setup, int strobe, int hold)
{
    int i, divi = ns / 2;

    smi_cs  = (SMI_CS_REG *) REG32(smi_regs, SMI_CS);
    smi_l   = (SMI_L_REG *)  REG32(smi_regs, SMI_L);
    smi_a   = (SMI_A_REG *)  REG32(smi_regs, SMI_A);
    smi_d   = (SMI_D_REG *)  REG32(smi_regs, SMI_D);
    smi_dmc = (SMI_DMC_REG *)REG32(smi_regs, SMI_DMC);
    smi_dsr = (SMI_DSR_REG *)REG32(smi_regs, SMI_DSR0);
    smi_dsw = (SMI_DSW_REG *)REG32(smi_regs, SMI_DSW0);
    smi_dcs = (SMI_DCS_REG *)REG32(smi_regs, SMI_DCS);
    smi_dca = (SMI_DCA_REG *)REG32(smi_regs, SMI_DCA);
    smi_dcd = (SMI_DCD_REG *)REG32(smi_regs, SMI_DCD);
    smi_cs->value = smi_l->value = smi_a->value = 0;
    smi_dsr->value = smi_dsw->value = smi_dcs->value = smi_dca->value = 0;
    if (*REG32(clk_regs, CLK_SMI_DIV) != divi << 12)
    {
        *REG32(clk_regs, CLK_SMI_CTL) = CLK_PASSWD | (1 << 5);
        usleep(10);
        while (*REG32(clk_regs, CLK_SMI_CTL) & (1 << 7)) ;
        usleep(10);
        *REG32(clk_regs, CLK_SMI_DIV) = CLK_PASSWD | (divi << 12);
        usleep(10);
        *REG32(clk_regs, CLK_SMI_CTL) = CLK_PASSWD | 6 | (1 << 4);
        usleep(10);
        while ((*REG32(clk_regs, CLK_SMI_CTL) & (1 << 7)) == 0) ;
        usleep(100);
    }
    if (smi_cs->seterr)
        smi_cs->seterr = 1;
    smi_dsr->rsetup = smi_dsw->wsetup = setup;
    smi_dsr->rstrobe = smi_dsw->wstrobe = strobe;
    smi_dsr->rhold = smi_dsw->whold = hold;
    smi_dmc->panicr = smi_dmc->panicw = 8;
    smi_dmc->reqr = smi_dmc->reqw = REQUEST_THRESH;
    smi_dsr->rwidth = smi_dsw->wwidth = width;
    for (i=0; i<LED_NCHANS; i++)
        gpio_mode(LED_D0_PIN+i, GPIO_ALT1);
}

static void setup_smi_dma(MEM_MAP *mp, int nsamp)
{
    DMA_CB *cbs = mp->virt;

    txdata = (TXDATA_T *)(cbs+1);
    smi_dmc->dmaen = 1;
    smi_cs->enable = 1;
    smi_cs->clear  = 1;
    smi_cs->pxldat = 1;
    smi_l->len     = nsamp * sizeof(TXDATA_T);
    smi_cs->write  = 1;

    enable_dma(DMA_CHAN);
    cbs[0].ti      = DMA_DEST_DREQ | (DMA_SMI_DREQ << 16) | DMA_CB_SRCE_INC | DMA_WAIT_RESP;
    cbs[0].tfr_len = nsamp * sizeof(TXDATA_T);
    cbs[0].srce_ad = MEM_BUS_ADDR(mp, txdata);
    cbs[0].dest_ad = REG_BUS_ADDR(smi_regs, SMI_D);
}

static void start_smi(MEM_MAP *mp)
{
    DMA_CB *cbs = mp->virt;
    start_dma(mp, DMA_CHAN, &cbs[0], 0);
    smi_cs->start = 1;
}

static void rgb_txdata(int *rgbs, TXDATA_T *txd)
{
    int i, n, msk;
    for (n = 0; n < LED_NBITS; n++) {
        msk = n==0 ? 0x8000 : n==8 ? 0x800000 : n==16 ? 0x80 : msk>>1;
        txd[0] = (TXDATA_T)0xffff;
        txd[1] = txd[2] = 0;
        for (i = 0; i < LED_NCHANS; i++)
            if (rgbs[i] & msk) txd[1] |= (1 << i);
        txd += BIT_NPULSES;
    }
}

void swap_bytes(void *data, int len)
{
    uint16_t *wp = (uint16_t *)data;

    len = (len + 1) / 2;
    while (len-- > 0)
    {
        *wp = __builtin_bswap16(*wp);
        wp++;
    }
}

// Free memory segments and exit
void terminate(int sig)
{
    int i;

    printf("Closing\n");
    if (gpio_regs.virt)
    {
        for (i=0; i<LED_NCHANS; i++)
            gpio_mode(LED_D0_PIN+i, GPIO_IN);
    }
    if (smi_regs.virt)
        *REG32(smi_regs, SMI_CS) = 0;
    stop_dma(DMA_CHAN);
    unmap_periph_mem(&vc_mem);
    unmap_periph_mem(&smi_regs);
    unmap_periph_mem(&dma_regs);
    unmap_periph_mem(&gpio_regs);
    exit(0);
}
