#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/time.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"
#include "inc/font.h"
#include "string.h"


// definições para uso do display oled integrado
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

// Pinos dos perifericos
const uint8_t LED_R=13, LED_B=12, LED_G=11;
const uint8_t BOTAO_A=5,BOTAO_B=6,BOTAO_JYK=22;

// Variavél para registro de tempo e controle de bounce da interrupção
static volatile uint32_t tempo_anterior = 0;

// Inicializa a estrutura do display
ssd1306_t ssd; 


// protótipos de funções
void inicializar_leds();
void inicializar_botoes();
void set_rgb(char cor,bool ativa);
void atualizar_display(char msg);
static void gpio_irq_handler(uint gpio, uint32_t events);
void inicializar_display_oled();
void pwm_setup(uint8_t PINO);
void set_pwm_dc(uint16_t duty_cycle, uint8_t PINO);

int main()
{
    stdio_init_all();

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}


/*
|   Função inicializar_leds
|   Configura os pinos da LED RGB como saída
*/
void inicializar_leds(){
    // led vermelha
    gpio_init(LED_R);
    gpio_set_dir(LED_R,GPIO_OUT);
    // led verde
    gpio_init(LED_G);
    gpio_set_dir(LED_G,GPIO_OUT);
    // led azul
    gpio_init(LED_B);
    gpio_set_dir(LED_B,GPIO_OUT);
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
    //botão C
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
            
        }
        else if(gpio == BOTAO_B){
        }
        else if(gpio == BOTAO_JYK){
            printf("Modo bootsel\n");
            reset_usb_boot(0,0);
        }
        atualizar_display('\0');
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