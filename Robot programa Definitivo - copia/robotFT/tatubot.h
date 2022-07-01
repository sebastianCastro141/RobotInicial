/*----------------------------------------------------------
    Librería para crear robot Tatu-bot
  ----------------------------------------------------------*/
#ifndef TATUBOT_H
#define TATUBOT_H
#include <Arduino.h>

// Motor 0
#define IN1 3//4 // 3 
#define IN2 4//5 // 2
#define ENA 2//3  // 5
// Motor 1
#define IN3 5//6 // 8 
#define IN4 6//23 // 7
#define ENB 7//7 // 9
// Matriz de Leds
#define DIN 44 // 11 
#define CLK 47 // 13 
#define CS 45 // 10
#define MATRICES 2
// Pulsadores
#define PIN_INICIO_ORDENES A0
#define PIN_FIN_ORDENES A2
#define PIN_ORDEN_AVANZAR A1
#define PIN_ORDEN_REVERSA A4
#define PIN_ORDEN_GIRO_DERECHA A3
#define PIN_ORDEN_GIRO_IZQUIERDA A5
#define PIN_RESET A6

//Led 1 Derecha mirando desde atras
#define PIN_R_A 8
#define PIN_G_A 9
#define PIN_B_A 10
//Led 2 Izquierda mirando desde atras
#define PIN_R_B 11
#define PIN_G_B 12
#define PIN_B_B 13
//sensores 
#define SENSOR0 28 // Derecha mirando desde atras
#define SENSOR1 29 // Izquierda mirando desde atras   
#define SENSORDERECHO  63 //verde va con motor 0 o sea der
#define SENSORIZQUIERDO 62 //azul va con motor 1 o sea izq

#define CANTIDAD_ORDENES 20

class Led {
  private:
    byte ledR;
    byte ledG;
    byte ledB;

   public:
    Led(byte ledR, byte ledG, byte ledB);
    void iniciar();
    void rgb(char color);
    };


class Motor {
  private:
    byte pinAtras;
    byte pinAdelante;
    byte pinVelocidad;
    
  public:
    Motor(byte pinAtras, byte pinAdelante, byte pinVelocidad);
    void iniciar();
    void adelante(int velocidad);
    void atras(int velocidad);
    void parar();
};


class Robot {
  private:
    int listaOrdenes[10];
  public:
    byte pinEscucharOrdenes = PIN_INICIO_ORDENES;
    byte pinAvanzar = PIN_ORDEN_AVANZAR;
    byte pinReversa = PIN_ORDEN_REVERSA;
    byte pinGiroDerecha = PIN_ORDEN_GIRO_DERECHA;
    byte pinGiroIzquierda = PIN_ORDEN_GIRO_IZQUIERDA;
    byte pinFinOrdenes = PIN_FIN_ORDENES;
    byte pinReset = PIN_RESET;

    
    void iniciar();
    void revisarNivelDeBateria();
    void avisarBajaBateria();
    void avisarMediaCapacidad();
    void iniciarDFPlayerMini();
    void reproducirSonido(int sonido, int tiempo);
    void avanzar(int velocidad);//, int velocidad2
    void movimientoAdelante(int velocidad);//, int velocidad2
    void terminarMovimiento(void (*funcion));
    void retroceder(int velocidad);//, int velocidad2
    void movimientoReversa(int velocidad);//, int velocidad2
    void girarDerecha();
    void movimientoDerecha();
    void girarIzquierda();
    void movimientoIzquierda();
    void pararRobot();
    void detectarLineaNegraEIgualar(int velocidad);
    void adelantarMotorAtrasado0(int velocidad);
    void adelantarMotorAtrasado1(int velocidad);
    void enderezarMotoresDesdeLineaNegra(int velocidad);
    void enderezarSiSePasanLosMotoresABlanco(int velocidad);
    void detectarLineaNegraEIgualarEnReversa(int velocidad);
    void enderezarSiSePasanLosMotoresABlancoEnReversa(int velocidad);
    void enderezarMotoresDesdeLineaNegraEnReversa(int velocidad);
    void moverMotor0HastaLineaNegra(int velocidad);
    void moverMotor1HastaLineaNegra(int velocidad);
    void escucharOrdenes();
    void ejecutarTodasLasOrdenes();
    void ejecutarOrden(byte caso);
    void contador(); 
    void autocorregir();
    void igualarRuedaAdelantada0();
    void igualarRuedaAdelantada1();
    void autocorregirReversa();
    void igualarRuedaAdelantadaReversa0();
    void igualarRuedaAdelantadaReversa1();
    void calcularCasosDeVelocidad();
    void dibujarCarita(byte ojos[8], byte boca[8]);
    void dibujarCaritaFeliz();
    void dibujarCaritaSorprendida();
    void dibujarCaritaEntusiasmada();
    void dibujarCaritaGuiniando();
    void dibujarCaritaTriste();
    void dibujarCaritaEnojada();
    void dibujarCaritaDormido();
    void despertar();
};

#endif
