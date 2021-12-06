//Bibliotecas utilizadas
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "ESPino32CAM.h"
#include "ESPino32CAM_QRCode.h"
#include <WiFi.h>                          //importa biblioteca para conectar esp32 com wifi
#include <FirebaseESP32.h>         //importa biblioteca para esp32 se comunicar com firebase
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_MLX90614.h>  //importa biblioteca do sensor de temperatura
#include "time.h"
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define WIFI_SSID "Redmi Note 7"
#define WIFI_PASSWORD "00000000"
#define API_KEY ""

#define PIN_MOTOR 12
#define PIN_IR 13
#define PIN_CATRACA 2
//Caminho do bd
#define DATABASE_URL ""

//Define os pinos da câmera
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define flash 4

ESPino32CAM cam;   //Objeto para captura de imagem
ESPino32QRCode qr; //Objeto para decoficação da imagem

Adafruit_MLX90614 mlx = Adafruit_MLX90614(); //objeto sensor

LiquidCrystal_I2C lcd(0x27, 16, 2); //endereço i2c sensor
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP); //Data e hora

String formattedDate; //Data
int conta = 0; //Cont para qtde de elementos no BD
int trava;

FirebaseData fbdo; //objeto para inclusão no bd
FirebaseData fbdo1;
FirebaseData fbdo2;

FirebaseAuth auth; //autentificação no bd
FirebaseConfig config;

void setup() {

  Serial.begin(115200);

  lcd.begin(); //inicialização LCD
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Iniciando");
  lcd.clear();

  if (!mlx.begin()) { //inicalização sensor
    lcd.setCursor(0, 0);
    lcd.print("Erro no sensor");
    lcd.setCursor(0, 1);
    lcd.print("de temperatura!");
    lcd.clear();
    while (1);
  };

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); //inicialização Wi-fi
  Serial.print("Conectando ao Wifi: ");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    lcd.setCursor(0, 0);
    lcd.print("CONECTANDO AO");
    lcd.setCursor(0, 1);
    lcd.print("WIFI");
    delay(300);
  }
  Serial.println();
  Serial.print("Conectado com o IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  config.api_key = API_KEY; //Chave de acesso ao BD
  config.database_url = DATABASE_URL; //URL BD

  config.token_status_callback = tokenStatusCallback; //Status BD

  Firebase.begin(DATABASE_URL, API_KEY);

  Firebase.reconnectWiFi(true);

  timeClient.begin();
  timeClient.setTimeOffset(-10800); //GMT BRASIL (data e hora servidor)

  pinMode(flash, OUTPUT);
  digitalWrite(flash, LOW); //Desliga o flash

  //Configura os pinos da câmera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 4;
  config.fb_count = 1;


  esp_err_t err = esp_camera_init(&config); //Inicialização da câmera

  if (err != ESP_OK) {
    Serial.printf("O início da câmera falhou com erro 0x%x", err);//Informa erro se a câmera não for iniciada corretamente
    delay(1000);
    ESP.restart();//Reinicia o ESP
  }

  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_IR, INPUT);
  pinMode(PIN_CATRACA, OUTPUT);

  qr.init(&cam); //Inicializa o objeto de decodificação
  sensor_t *s = cam.sensor();
  s->set_framesize(s, FRAMESIZE_CIF);
  s->set_whitebal(s, true);


  Serial.println();
  Serial.println("Aguardando código");
}

void loop()
{
  unsigned long pv_time  = millis();
  camera_fb_t *fb = cam.capture(); //Captura a imagem
  if (!fb)
  {
    Serial.println("Falha na captura da imagem");
    return;
  }
  dl_matrix3du_t *rgb888, *rgb565;
  if (cam.jpg2rgb(fb, &rgb888))
  {
    rgb565 = cam.rgb565(rgb888);
  }
  cam.clearMemory(rgb888);
  cam.clearMemory(rgb565);
  dl_matrix3du_t *image_rgb;
  if (cam.jpg2rgb(fb, &image_rgb))
  {
    cam.clearMemory(fb);

    qrResoult res = qr.recognition(image_rgb); //Faz decodificação da imagem contendo os dados

    if (res.status) //Se conseguir decodificar a imagem
    {
      if (trava == 0) {
        trava = 1;
        String leitura = res.payload;//Variável para mostrar os dados contidos no QR Code
        Serial.println();
        Serial.println(leitura);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SEJA BEM VINDO,");
        lcd.setCursor(0, 1);
        lcd.print(leitura);

        if (BuscarUsuario(leitura)) {
          double valorcalc = AferirTemp();
          Serial.println(valorcalc);
          if (!ValidaTemp(valorcalc)) { //validação da temperatura recebida
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("ERRO DE LEITURA");
            delay(1500);
            lcd.clear();
          } else {
            // chama ligação do motor e inclui registro
            AcionaMotor();
            IncluirRegistro(leitura, valorcalc);
          }
        }
      }
    }
    else { //Se não aguarda receber código
      trava = 0;
      Serial.println();
      lcd.setCursor(0, 0);
      lcd.print("POSICIONE O     ");
      lcd.setCursor(0, 1);
      lcd.print("QRCODE NA CAMERA");
      Serial.println("AGUARDANDO QR CODE");

    }
  }
  cam.clearMemory(image_rgb); //Apaga imagem para receber uma nova imagem
}

bool BuscarUsuario(String leitura) {

  FirebaseJson json;
  QueryFilter query;
  query.orderBy("nome");
  query.equalTo(leitura);

  if (Firebase.getJSON(fbdo, "/cadastro/usuario", query))
  {
    Serial.println(fbdo.jsonString());
    if (fbdo.jsonString() == "{}")
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("USUARIO NAO");
      lcd.setCursor(0, 1);
      lcd.print("CADASTRADO!");
      query.clear();
      fbdo.clear();
      delay(2000);
      return false;
    }
    else {
      query.clear();
      fbdo.clear();
      return true;
    }
  }
  else
  { //erro para executar a query
    Serial.println(fbdo.errorReason());
  }
}

void AcionaMotor() {
  lcd.clear();
  while (digitalRead(PIN_IR) != LOW) {
    lcd.setCursor(0, 0);
    lcd.print("POSICIONE A MAO");
    lcd.setCursor(0, 1);
    lcd.print("P/ O ALCOOL");
    delay(800);
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LIBERANDO ALCOOL");
  digitalWrite(PIN_MOTOR, HIGH);
  delay(3000);
  digitalWrite(PIN_MOTOR, LOW);

}
void IncluirRegistro(String leitura, double tempMedia) {
  lcd.setCursor(0, 0);
  lcd.print("REGISTRANDO SEUS");
  lcd.setCursor(0, 1);
  lcd.print("DADOS.");

  if (Firebase.getInt(fbdo1, "/cont")) {
    if (fbdo1.dataType() == "int") {
      conta = fbdo1.intData();
      conta++;
      Firebase.setInt(fbdo1, "/cont", conta);
    }
  } else {
    conta = 1;
    Firebase.setInt(fbdo1, "/cont", conta);
  }
  fbdo1.clear();
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }
  formattedDate = timeClient.getFormattedDate();
  Serial.println(formattedDate);

  bool acesso = ValidaAcesso(tempMedia); //definição se o usuário acessou ou não
  String myString = String(conta);

  //envio das informações ao BD
  Firebase.setString(fbdo2, "/dados/" + myString + "/nome", leitura);

  Firebase.setString(fbdo2, "/dados/" + myString + "/data", formattedDate);

  Firebase.setFloat(fbdo2, "/dados/" + myString + "/temp", tempMedia);

  Firebase.setBool(fbdo2, "/dados/" + myString + "/acesso", acesso);

  fbdo2.clear();
  lcd.clear();
}

double AferirTemp() {
  double tempMedia1 = 0;
  double tempMedia = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("APROXIME O PULSO");
  lcd.setCursor(0, 1);
  lcd.print("NO SENSOR");

  while (1)
  {
    tempMedia =  mlx.readObjectTempC();
    if (tempMedia > 28) {
      tempMedia1 = 0;
      delay(1000); //tempo pra posicionar a mão no sensor
      for (int i = 0; i < 10; i++) {
        tempMedia1 = (tempMedia1 + mlx.readObjectTempC());
      }
      break;
    }
  }
  tempMedia1 = (tempMedia1 / 10); //media temperatura
  tempMedia1 =  0.0974 * (tempMedia1 * tempMedia1) - 6.038 * tempMedia1 + 129.32; // calibração sensor

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SUA TEMPERATURA:");
  lcd.setCursor(0, 1);
  lcd.print((tempMedia1), 1);
  lcd.print(" oC");
  delay(2000);

  return tempMedia1;
}

bool ValidaTemp(double tempMedia) {
  if (tempMedia < 25 || tempMedia > 42)
    return false;
  return true;
}

bool ValidaAcesso(double tempMedia) { //verifica se o usuário pode entrar
  if (tempMedia > 37.5) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ENTRADA NEGADA,");
    lcd.setCursor(0, 1);
    lcd.print("USUARIO FEBRIL");
    delay(5000);
    return false;
  }
  digitalWrite(PIN_CATRACA, HIGH);
  delay(200);
  digitalWrite(PIN_CATRACA, LOW);
  return true;
}
