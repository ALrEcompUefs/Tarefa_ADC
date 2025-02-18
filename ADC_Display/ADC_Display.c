#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/time.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"
#include "inc/font.h"
#include "string.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

// definições para uso do display oled integrado
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

// paramêtros padrão para o joystick
#define PADRAO_VRX 1890
#define PADRAO_VRY 2050
//valor máximo do contador - WRAP
#define WRAP_PERIOD  4096
//divisor do clock para o PWM 
#define PWM_DIVISER  25.4313151f 

// Pinos dos perifericos
const uint8_t LED_R=13, LED_B=12, LED_G=11;
const uint8_t BOTAO_A=5,BOTAO_B=6,BOTAO_JYK=22;
const uint8_t VRX = 26, VRY=27;
// Variavél para registro de tempo e controle de bounce da interrupção
static volatile uint32_t tempo_anterior = 0;

// variável de controle de leitura do adc
static volatile uint16_t valor_x,valor_y;
// Inicializa a estrutura do display
ssd1306_t ssd; 

// protótipos de funções
void inicializar_leds();
void inicializar_botoes();
void inicializar_joystick();
void leitura_adc();
void obter_nivel_pwm();
void atualizar_display(char msg);
static void gpio_irq_handler(uint gpio, uint32_t events);
void inicializar_display_oled();
void pwm_setup(uint8_t PINO);
void set_pwm_dc(uint16_t duty_cycle, uint8_t PINO);

int main()
{
    stdio_init_all();
    inicializar_leds();
    inicializar_botoes();
    inicializar_joystick();
    inicializar_display_oled();
    adc_init();

    // configura interrupções
    gpio_set_irq_enabled_with_callback(BOTAO_A,GPIO_IRQ_EDGE_FALL,true,&gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_B,GPIO_IRQ_EDGE_FALL,true,&gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_JYK,GPIO_IRQ_EDGE_FALL,true,&gpio_irq_handler);
    while (true) {
        leitura_adc();
        obter_nivel_pwm();
        printf("Leitura do Joystick!\n");
        printf("Eixo X:%d Eixo Y:%d\n",valor_x,valor_y);
        sleep_ms(1000);
    }
}


/*
|   Função inicializar_leds
|   Configura os pinos da LED RGB como saída
*/
void inicializar_leds(){
    // led vermelha
    pwm_setup(LED_R);
    pwm_setup(LED_G);
    pwm_setup(LED_B);
}

/*
|   Função de inicialização dos botões
|   Configura os botões A,B e do joystick como entrada em modo Pull-up
*/
void inicializar_botoes(){
    //botão A
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A,GPIO_IN);
    gpio_pull_up(BOTAO_A);
    //botão B
    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B,GPIO_IN);
    gpio_pull_up(BOTAO_B);
    
}

void inicializar_joystick(){
    // Inicializa pinos do joystick como pinos do ADC
    adc_gpio_init(VRX);
    adc_gpio_init(VRY);
    //botão do joystick
    gpio_init(BOTAO_JYK);
    gpio_set_dir(BOTAO_JYK,GPIO_IN);
    gpio_pull_up(BOTAO_JYK);
}
/*
|   Função inicialzar display oled
|   Configura e inicializa o display oled ssd1306 para sua utilização
|   A comunicação é feita utilizando i2c
*/
void inicializar_display_oled(){
    // I2C Initialisation. Using it at 400Khz.
  i2c_init(I2C_PORT, 400 * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_pull_up(I2C_SDA); // Pull up the data line
  gpio_pull_up(I2C_SCL); // Pull up the clock line
  //ssd1306_t ssd; // Inicializa a estrutura do display
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd); // Configura o display
  ssd1306_send_data(&ssd); // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);

}


/*
|   Função gpio_irq_handler
|   Função de callback para tratamento de interrupção da GPIO
|   Implementa um debouncer via software e controla o estado das leds azul e verde
|   além de ativar o modo bootsel
*/
static void gpio_irq_handler(uint gpio, uint32_t events){
    // obtém tempo atual da execução do programa
    uint32_t tempo_atual = to_us_since_boot(get_absolute_time());
    // com o botão pressionado por pelo menos 200ms
    if(tempo_atual-tempo_anterior > 200000){
        tempo_anterior= tempo_atual;
        // executa tratamento da interrupção
        if(gpio == BOTAO_A){
            printf("teste\n");
        }
        else if(gpio == BOTAO_B){
            printf("teste2\n");
        }
        else if(gpio == BOTAO_JYK){
            printf("Modo bootsel\n");
            reset_usb_boot(0,0);
        }
    }
}


//função para configurar o módulo PWM
void pwm_setup(uint8_t PINO)
{
    // Com o clock base de 125Mhz WRAP de 10000 e divisor de 125
    // O clock calculado para o pwm é de 50Hz
    // outras configurações de parâmetro poderiam ser utilizadas para obter esta mesma frequência
    // Mas os requisitos da tarefa limitaram a escolha do valor do wrap

    gpio_set_function(PINO, GPIO_FUNC_PWM); //habilitar o pino GPIO como PWM

    uint slice = pwm_gpio_to_slice_num(PINO); //obter o canal PWM da GPIO

    pwm_set_wrap(slice, WRAP_PERIOD); //definir o valor de wrap
    pwm_set_clkdiv(slice, PWM_DIVISER); //define o divisor de clock do PWM
    pwm_set_enabled(slice, true); //habilita o pwm no slice correspondente
}

/*
|   Função set_pwm_dc
|   Configura o nível do duty cycle para o pwm
*/
void set_pwm_dc(uint16_t duty_cycle, uint8_t PINO){
    uint slice = pwm_gpio_to_slice_num(PINO);
    pwm_set_gpio_level(PINO, duty_cycle); //definir o cico de trabalho (duty cycle) do pwm
}
/*
|
|
*/
void leitura_adc(){
    // leitura do canal 0- eixox do joystick
    adc_select_input(0);
    sleep_us(2);
    valor_x=adc_read();
    // leitura do canal 1- eixo y do joystick
    adc_select_input(1);
    sleep_us(2);
    valor_y=adc_read();
}

/*
|
|
*/
void obter_nivel_pwm(){
    // calcula nivel pwm com base no valor do joystick
    if(valor_x == PADRAO_VRX){
        set_pwm_dc(0,LED_R);
    }
    else{
        // calcula o nivel
        uint16_t aux;
        valor_x > PADRAO_VRX? aux=valor_x-PADRAO_VRX:PADRAO_VRX-valor_x;
        // dobra o valor
        aux=aux*2;
        printf("PWM de X:%d\n",aux);
        set_pwm_dc(aux,LED_R);
    }

    if(valor_y == PADRAO_VRY){
        set_pwm_dc(0,LED_B);
    }
    else{
        // calcula o nivel
        uint16_t aux;
        valor_y > PADRAO_VRY? aux=valor_y-PADRAO_VRY:PADRAO_VRY-valor_y;
        // dobra o valor
        aux=aux*2;
        printf("PWM de Y:%d\n",aux);
        set_pwm_dc(aux,LED_B);
    }
    sleep_ms(500);
}