//`define ENABLE_DDR4
//`define ENABLE_HPS
//`define ENABLE_HDMI
//`define ENABLE_HSMC
//`define ENABLE_CAM

module top_de25_standard (
    input logic CLOCK0_50,
    input logic CLOCK1_50,
    input logic CLOCK2_50,

    input logic CPU_RESET_n,
    input logic [3:0] KEY,

    input logic [9:0] SW,

    output logic [9:0] LEDR,

    output logic [6:0] HEX0,
    output logic [6:0] HEX1,
    output logic [6:0] HEX2,
    output logic [6:0] HEX3,
    output logic [6:0] HEX4,
    output logic [6:0] HEX5,

    output logic DRAM_CLK,
    output logic DRAM_CKE,
    output logic [12:0] DRAM_ADDR,
    output logic [1:0] DRAM_BA,
    output logic DRAM_LDQM,
    output logic DRAM_UDQM,
    output logic DRAM_CS_n,
    output logic DRAM_WE_n,
    output logic DRAM_CAS_n,
    output logic DRAM_RAS_n,
    inout logic [15:0] DRAM_DQ,

`ifdef ENABLE_DDR4
    output logic [16:0] DDR4_A,
    output logic [1:0] DDR4_BA,
    output logic [0:0] DDR4_BG,
    output logic DDR4_CK,
    output logic DDR4_CK_n,
    output logic DDR4_CKE,
    output logic DDR4_CS_n,
    output logic DDR4_RESET_n,
    output logic DDR4_ODT,
    output logic DDR4_PAR,
    output logic DDR4_ACT_n,
    inout logic [3:0] DDR4_DQS,
    inout logic [3:0] DDR4_DQS_n,
    inout logic [31:0] DDR4_DQ,
    inout logic [3:0] DDR4_DBI_n,
    input logic DDR4_REFCLK_p,
    input logic DDR4_ALERT_n,
    input logic DDR4_RZQ,
`endif

    output logic TD_RESET_n,
    input logic TD_CLK27,
    input logic TD_HS,
    input logic TD_VS,
    input logic [7:0] TD_DATA,

`ifdef ENABLE_HDMI
    output logic HDMI_TX_CLK,
    output logic HDMI_TX_HS,
    output logic HDMI_TX_VS,
    output logic [23:0] HDMI_TX_D,
    output logic HDMI_TX_DE,
    inout logic HDMI_LRCLK,
    inout logic HDMI_MCLK,
    inout logic HDMI_SCLK,
    inout logic HDMI_I2S,
    input logic HDMI_TX_INT,
`endif

    output logic AUD_XCK,
    output logic AUD_DACDAT,
    inout logic AUD_BCLK,
    inout logic AUD_ADCLRCK,
    inout logic AUD_DACLRCK,
    input logic AUD_ADCDAT,

    output logic ADC_SCK,
    output logic ADC_SDI,
    output logic ADC_CS_n,
    input logic ADC_SDO,

    inout logic FPGA_I2C_SCLK,
    inout logic FPGA_I2C_SDAT,

    inout logic [35:0] GPIO_D,

`ifdef ENABLE_HSMC
    output logic [3:0] HSMC_GTS_TX_p,
    output logic [3:0] HSMC_GTS_TX_n,
    output logic HSMC_I2C_SCL,
    inout logic HSMC_I2C_SDA,
    inout logic [1:0] HSMC_B5B_D,
    inout logic [76:0] HSMC_D,
    input logic [3:0] HSMC_GTS_RX_p,
    input logic [3:0] HSMC_GTS_RX_n,
    input logic HSMC_GTS_RX_REFCLK_p,
    input logic [2:0] HSMC_CLK_IN,
    input logic HSMC_GTS_REFCLK_p,
`endif

`ifdef ENABLE_CAM
    inout logic CAM_GPIO,
    input logic CAM_CLK_p,
    input logic CAM_CLK_n,
    input logic [1:0] CAM_D_p,
    input logic [1:0] CAM_D_n,
    input logic CAM_RZQ1,
`endif

`ifdef ENABLE_HPS
    output logic HPS_ENET_MDC,
    output logic HPS_ENET_TX_CLK,
    output logic HPS_ENET_TX_CTL,
    output logic [3:0] HPS_ENET_TX_DATA,
    output logic HPS_LCM_SPIM_CLK,
    output logic HPS_LCM_SPIM_MOSI,
    output logic HPS_LCM_SPIM_SS,
    output logic HPS_SD_CLK,
    output logic HPS_UART_TX,
    output logic HPS_USB_STP,
    inout logic HPS_ENET_MDIO,
    inout logic [1:0] HPS_GPIO,
    inout logic HPS_GSENSOR_INT,
    inout logic HPS_I2C_SCL,
    inout logic HPS_I2C_SDA,
    inout logic HPS_KEY,
    inout logic HPS_LCM_BK,
    inout logic HPS_LCM_D_C,
    inout logic HPS_LCM_RST_n,
    inout logic HPS_LED,
    inout logic HPS_SD_CMD,
    inout logic [3:0] HPS_SD_DATA,
    inout logic [7:0] HPS_USB_DATA,
    input logic HPS_CLK_25,
    input logic HPS_ENET_RX_CLK,
    input logic HPS_ENET_RX_CTL,
    input logic [3:0] HPS_ENET_RX_DATA,
    input logic HPS_UART_RX,
    input logic HPS_USB_CLK,
    input logic HPS_USB_DIR,
    input logic HPS_USB_NXT,
`endif

    output logic IRDA_TXD,
    input logic IRDA_RXD
);


/* Local variables and signals */

logic clk_100, rst_n, ninit_done;

logic [31:0] counter;


/* Signals assignments */

assign clk = CLOCK0_50;
assign rst_n = KEY[0] & ~ninit_done;

assign LEDR = ~counter[31:22];

assign HEX0 = {7{1'b1}};
assign HEX1 = {7{1'b1}};
assign HEX2 = {7{1'b1}};
assign HEX3 = {7{1'b1}};
assign HEX4 = {7{1'b1}};
assign HEX5 = {7{1'b1}};


/* Submodules */

reset_release u_reset_release (
    .ninit_done
);

pll u_pll (
    .refclk(CLOCK0_50),
    .rst(~rst_n),

    .outclk_0(clk_100)
);

BOARD_MANAGEMENT u_BOARD_MANAGEMENT (
    .clk_50(CLOCK0_50),
    .reset_n(rst_n),

    .Auto_Fan_Speed(1'b1),

    .I2C_READY(),
    .I2C_SCL(FPGA_I2C_SCLK),
    .I2C_SDA(FPGA_I2C_SDAT)
);


/* Internal logic */

always_ff @(posedge clk_100 or negedge rst_n) begin
    if (!rst_n)
        counter <= 32'b0;
    else
        counter <= counter + 1;
end


endmodule
