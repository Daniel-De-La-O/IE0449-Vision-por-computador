//Mapping and analyzing the position and motion of a
//moving objecct in a scene with a static monocular camera
//
//Created by Daniel De La O Rojas on 16/10/23.
//

//Este programa lee de un archivo de texto llamado current_control_parameters.txt, 
//y con esa información realiza varias funciones especificadas abajo. Al final genera
//un archivo de texto denominado resultados.txt donde se encuentran los resultados de
//la ejecución de dichas funciones.

//Se incluyo el siguiente header de la biblioteca
//estandar de C para operaciones de entrada y salida
#include <stdio.h>

//Se incluyo el siguiente header de la biblioteca
//estandar de C para gestion de memoria dinamica,
//control de procesos y otras
#include <stdlib.h>

//Se incluyo el siguiente header debido a que usaremos
//funciones matematicas
#include <math.h>

//Prototipos de las funciones

void danObtenerMatrizDeRotacionR();
void danObtenerMatrizDeRotacionR2(double RokcX, double RokcY, double RokcZ);
void danObtenerMatrizDeRotacionR3();
void danObtenerMatrizDeRotacionR4(double Rckmx, double Rckmy, double Rckmz);
void danObtenerMatrizDeRotacionR5();
void danObtenerMatrizDeRotacionR6(double DRo2cX, double DRo2cY, double DRo2cZ);
double danConvertirDeGradosARadianes (double angle);
void danLeerParametrosDeControlDeArchivoDeTexto();
void danSalvarResultadosEnArchivoDeTexto();

//La siguiente definicion describe el contenedor que
//usaremos para guardar los parametros de control
//de flujo del programa

struct contenedor_de_parametros_de_control
{
    //Posicion de la cámara con respecto el sistema
    //de coordenadas del mundo
    double Gckmx;
    double Gckmy;
    double Gckmz;
    //Orientacion de la cámara respecto el sistema
    //de coordenadas del mundo
    double Rckmx;
    double Rckmy;
    double Rckmz;
    //Posicion del objeto con respecto el sistema
    //de coordenadas de la cámara en el instante k
    double GokcX;
    double GokcY;
    double GokcZ;
    //Orientacion del objeto con respecto el sistema
    //de coordenadas de la cámara en el instante k
    double RokcX;
    double RokcY;
    double RokcZ;
    //Posicion del punto H respecto al sistema
    //de coordenadas del objeto
    double Hokor;
    double Hokos;
    double Hokot;
    //Cambio de posición del objeto respecto al sistema
    //de coordenadas de la cámara entre el instante k y k+1
    double DTo2cX;
    double DTo2cY;
    double DTo2cZ;
    //Cambio de orientación del objeto respecto al sistema
    //de coordenadas de la cámara entre el instante k y k+1
    double DRo2cX;
    double DRo2cY;
    double DRo2cZ;
    //Distancia focal
    double f;
    //Píxeles de largo y de alto repectivamente
    double Nfu;
    double Nfv;
    //Número de sensores por línea y por columna respectivamente
    double Ncu;
    double Ncv;
    //Distancia entre sensores a lo largo del eje horizontal y la 
    //distancia entre sensores a lo largo del eje vertical respectivamente.
    double Du;
    double Dv;
};
 
//La siguiente definicion describe el contenedor que
//usaremos para guardar resultados

struct contenedor_de_resultados
{
    //Matriz de rotacion del objeto con respecto a la cámara en el instante k. Dos formas equivalentes de definirla
    double R[9]; //Fijo, mediante arreglo. La reserva y liberacion de memoria es automatica
    double *R2;  //Dinamico, mediante puntero. Requiere reserva y liberación manual de memoria

    //Matriz de rotacion de la cámara con respecto al mundo en el instante k.
    double R3[9]; 
    double *R4; 

    //Matriz de rotación que describe el cambio de orientación del sistema de coordenadas 
    //del objeto entre los instantes de tiempo k y k+1, respecto al sistema de coordenadas de la cámara
    double R5[9]; 
    double *R6;  

    //Posicion del punto respecto al sistema de coordenadas
    //local de la cámara en el instante de tiempo k
    double Hokcx;
    double Hokcy;
    double Hokcz;

    //Posicion bidimensional del punto con respecto al sistema de
    //coordenadas local del plano de la cámara en el instante de tiempo k
    double huk;
    double hvk;

    //Posicion bidimensional del punto con respecto al sistema de
    //coordenadas local de la imagen en el instante de tiempo k
    double cik;
    double rik;

    //Posicion del punto respecto al sistema de coordenadas
    //del mundo en el instante de tiempo k
    double Hokmx;
    double Hokmy;
    double Hokmz;

    //Posicion del punto respecto al sistema de coordenadas
    //local de la cámara en el instante de tiempo k+1
    double Ho2cX;
    double Ho2cY;
    double Ho2cZ;

    //Posicion bidimensional del punto con respecto al sistema de
    //coordenadas local del plano de la cámara en el instante de tiempo k+1
    double huk2;
    double hvk2;

    //Posicion bidimensional del punto con respecto al sistema de
    //coordenadas local de la imagen en el instante de tiempo k+1
    double cik2;
    double rik2;

    //6 parámetros que describen la pose del objeto con
    //respecto al sistema de coordenadas de la cámara en
    //el instante de tiempo k+1; separados en 3 parámetros
    //que describen la posicion del objeto con respecto el sistema
    //de coordenadas de la cámara en el instante k+1 y 3 parámetros
    //que describen la orientación del objeto con respecto el sistema
    //de coordenadas de la cámara en el instante k+1
    double Gok2cx;
    double Gok2cy;
    double Gok2cz;
    double Rok2cx;
    double Rok2cy;
    double Rok2cz;    

    //El vector bidimensional que describe el desplazamiento del punto sobre el plano de la cámara
    //entre los instantes de tiempo k y k+1, con respecto al sistema de coordenadas local del plano de la cámara
    double duk;
    double dvk;

    //El vector bidimensional que describe el desplazamiento del punto sobre el plano de la cámara
    //entre los instantes de tiempo k y k+1, con respecto al sistema de coordenadas de la imagen
    double dck;
    double drk;

};

//El siguiente puntero global apuntara al contenedor que
//usaremos para guardar los valores de control de flujo
//del programa que se leeran de un archivo de texto

struct contenedor_de_parametros_de_control *p_parametros;

//El siguiente puntero global apuntara al contenedor
//que usaremos para guardar resultados

struct contenedor_de_resultados *p_resultados;

//La siguiente variable global se usara como contador
//el numero de datos leidos

int numeroDeDatosLeidos=0;

//Constantes del programa

#define PI 3.141592652
#define DESPLIEGUE_MATRIZ_DE_ROTACION 1 //1: si, 0: no

//Inicio de programa principal
int main()
{
    //definición de variables locales
    int i; //contador
    double H[3]; //para guardar temporalmente los resultados
    double hk[2];
    double ak[2];
    double H2[3];
    double H3[3];
    double hk2[2];
    double ak2[2];
    double P[3];
    double P2[3];
    double D1[2];
    double D2[2];

    //Despliegue de autoría en el terminal
    printf("******************************************************************\n");
    printf("** IE-0449 Vision por Computador                                **\n");
    printf("** Daniel De La O Rojas                                         **\n");
    printf("** II-2023                                                      **\n");
    printf("******************************************************************\n");
    printf("\n");

    //Reservando e inicializando memoria de contenedor p_parametros
    p_parametros = (struct contenedor_de_parametros_de_control *)malloc(sizeof(struct contenedor_de_parametros_de_control));
    p_parametros->Gckmx=0.0;
    p_parametros->Gckmy=0.0;
    p_parametros->Gckmz=0.0;
    p_parametros->Rckmx=0.0;
    p_parametros->Rckmy=0.0;
    p_parametros->Rckmz=0.0;
    p_parametros->GokcX=0.0;
    p_parametros->GokcY=0.0;
    p_parametros->GokcZ=0.0;
    p_parametros->RokcX=0.0;
    p_parametros->RokcY=0.0;
    p_parametros->RokcZ=0.0;
    p_parametros->Hokor=0.0;
    p_parametros->Hokos=0.0;
    p_parametros->Hokot=0.0;
    p_parametros->DTo2cX=0.0;
    p_parametros->DTo2cY=0.0;
    p_parametros->DTo2cZ=0.0;
    p_parametros->DRo2cX=0.0;
    p_parametros->DRo2cY=0.0;
    p_parametros->DRo2cZ=0.0;
    p_parametros->f=0.0;
    p_parametros->Nfu=0.0;
    p_parametros->Nfv=0.0;
    p_parametros->Ncu=0.0;
    p_parametros->Ncv=0.0;
    p_parametros->Du=0.0;
    p_parametros->Dv=0.0;

    //Reservando e inicializando memoria de contenedor p_resultados
    p_resultados = (struct contenedor_de_resultados *)malloc(sizeof(struct contenedor_de_resultados));
    p_resultados->R2=NULL;
    p_resultados->R4=NULL;
    p_resultados->R6=NULL; 
    p_resultados->Hokcx=0.0;
    p_resultados->Hokcy=0.0;
    p_resultados->Hokcz=0.0;
    p_resultados->huk=0.0;
    p_resultados->hvk=0.0;
    p_resultados->cik=0.0;
    p_resultados->rik=0.0;
    p_resultados->Hokmx=0.0;
    p_resultados->Hokmy=0.0;
    p_resultados->Hokmz=0.0;
    p_resultados->Ho2cX=0.0;
    p_resultados->Ho2cY=0.0;
    p_resultados->Ho2cZ=0.0;
    p_resultados->huk2=0.0;
    p_resultados->hvk2=0.0;
    p_resultados->cik2=0.0;
    p_resultados->rik2=0.0;
    p_resultados->Gok2cx=0.0;
    p_resultados->Gok2cy=0.0;
    p_resultados->Gok2cz=0.0;
    p_resultados->Rok2cx=0.0;
    p_resultados->Rok2cy=0.0;
    p_resultados->Rok2cz=0.0;    
    p_resultados->duk=0.0;
    p_resultados->dvk=0.0;
    p_resultados->dck=0.0;
    p_resultados->drk=0.0;

    //Inicializando memoria de la matriz de rotacion R
    for (i=0;i<9;i++) { //i={0,1,2,3,4,5,6,7,8}
        p_resultados->R[i]=0.0;
    }
    //Reservando e inicializando memoria de la matriz de rotacion R2
    p_resultados->R2=(double *)malloc(9*sizeof(double));
    for (i=0;i<9;i++) { //i={0,1,2,3,4,5,6,7,8}
        p_resultados->R2[i]=0.0;
    }

    //Inicializando memoria de la matriz de rotacion R3
    for (i=0;i<9;i++) { //i={0,1,2,3,4,5,6,7,8}
        p_resultados->R3[i]=0.0;
    }
    //Reservando e inicializando memoria de la matriz de rotacion R4
    p_resultados->R4=(double *)malloc(9*sizeof(double));
    for (i=0;i<9;i++) { //i={0,1,2,3,4,5,6,7,8}
        p_resultados->R4[i]=0.0;
    }

    //Inicializando memoria de la matriz de rotacion R5
    for (i=0;i<9;i++) { //i={0,1,2,3,4,5,6,7,8}
        p_resultados->R5[i]=0.0;
    }
    //Reservando e inicializando memoria de la matriz de rotacion R6
    p_resultados->R6=(double *)malloc(9*sizeof(double));
    for (i=0;i<9;i++) { //i={0,1,2,3,4,5,6,7,8}
        p_resultados->R6[i]=0.0;
    }

    //Esta función lee los parámetros de control de flujo del
    //programa desde un archivo de texto y los almacena en el
    //contenedor p_parametros
    danLeerParametrosDeControlDeArchivoDeTexto();

    //Calculando matriz de rotacion del objeto con respecto a la cámara en el instante k
    danObtenerMatrizDeRotacionR();
    //Otra forma equivalente de calcular la matriz de rotacion del objeto con respecto a la cámara en el instante k
    danObtenerMatrizDeRotacionR2(p_parametros->RokcX, p_parametros->RokcY, p_parametros->RokcZ);
    //Calculando matriz de rotacion de la cámara con respecto al mundo en el instante k
    danObtenerMatrizDeRotacionR3();
    //Otra forma equivalente de calcular la matriz de rotacion de la cámara con respecto al mundo en el instante k
    danObtenerMatrizDeRotacionR4(p_parametros->Rckmx, p_parametros->Rckmy, p_parametros->Rckmz);
    //Calculando matriz de rotacion del objeto con respecto a la cámara en el instante k+1
    danObtenerMatrizDeRotacionR5();
    //Otra forma equivalente de calcular la matriz de rotacion del objeto con respecto a la cámara en el instante k+1
    danObtenerMatrizDeRotacionR6(p_parametros->DRo2cX, p_parametros->DRo2cY, p_parametros->DRo2cZ);
  

    //Desplegando matriz de rotacion R
    if (DESPLIEGUE_MATRIZ_DE_ROTACION==1) {
        printf("\n"); //Linea en blanco
        printf("Matriz de rotacion R\n");
        printf("   r1=%f r2=%f r3=%f\n", p_resultados->R[0],p_resultados->R[1],p_resultados->R[2]);
        printf("   r4=%f r5=%f r6=%f\n", p_resultados->R[3],p_resultados->R[4],p_resultados->R[5]);
        printf("   r7=%f r8=%f r9=%f\n", p_resultados->R[6],p_resultados->R[7],p_resultados->R[8]);
    }
    else {
        printf("\n");
        printf("No se desplegara matriz de rotacion R\n");
    }

    //Desplegando matriz de rotacion R2
    if (DESPLIEGUE_MATRIZ_DE_ROTACION==1) {
        printf("\n"); //Linea en blanco
        printf("Matriz de rotacion R2 (debe ser igual a R)\n");
        printf("   r1=%f r2=%f r3=%f\n", p_resultados->R2[0],p_resultados->R2[1],p_resultados->R2[2]);
        printf("   r4=%f r5=%f r6=%f\n", p_resultados->R2[3],p_resultados->R2[4],p_resultados->R2[5]);
        printf("   r7=%f r8=%f r9=%f\n", p_resultados->R2[6],p_resultados->R2[7],p_resultados->R2[8]);
    }
    else {
        printf("\n");
        printf("No se desplegara matriz de rotacion R2\n");
    }


    //Desplegando matriz de rotacion R3
    if (DESPLIEGUE_MATRIZ_DE_ROTACION==1) {
        printf("\n"); //Linea en blanco
        printf("Matriz de rotacion R3\n");
        printf("   r1=%f r2=%f r3=%f\n", p_resultados->R3[0],p_resultados->R3[1],p_resultados->R3[2]);
        printf("   r4=%f r5=%f r6=%f\n", p_resultados->R3[3],p_resultados->R3[4],p_resultados->R3[5]);
        printf("   r7=%f r8=%f r9=%f\n", p_resultados->R3[6],p_resultados->R3[7],p_resultados->R3[8]);
    }
    else {
        printf("\n");
        printf("No se desplegara matriz de rotacion R3\n");
    }

    //Desplegando matriz de rotacion R4
    if (DESPLIEGUE_MATRIZ_DE_ROTACION==1) {
        printf("\n"); //Linea en blanco
        printf("Matriz de rotacion R4 (debe ser igual a R3)\n");
        printf("   r1=%f r2=%f r3=%f\n", p_resultados->R4[0],p_resultados->R4[1],p_resultados->R4[2]);
        printf("   r4=%f r5=%f r6=%f\n", p_resultados->R4[3],p_resultados->R4[4],p_resultados->R4[5]);
        printf("   r7=%f r8=%f r9=%f\n", p_resultados->R4[6],p_resultados->R4[7],p_resultados->R4[8]);
    }
    else {
        printf("\n");
        printf("No se desplegara matriz de rotacion R4\n");
    }


    //Desplegando matriz de rotacion R5
    if (DESPLIEGUE_MATRIZ_DE_ROTACION==1) {
        printf("\n"); //Linea en blanco
        printf("Matriz de rotacion R5\n");
        printf("   r1=%f r2=%f r3=%f\n", p_resultados->R5[0],p_resultados->R5[1],p_resultados->R5[2]);
        printf("   r4=%f r5=%f r6=%f\n", p_resultados->R5[3],p_resultados->R5[4],p_resultados->R5[5]);
        printf("   r7=%f r8=%f r9=%f\n", p_resultados->R5[6],p_resultados->R5[7],p_resultados->R5[8]);
    }
    else {
        printf("\n");
        printf("No se desplegara matriz de rotacion R5\n");
    }

    //Desplegando matriz de rotacion R6
    if (DESPLIEGUE_MATRIZ_DE_ROTACION==1) {
        printf("\n"); //Linea en blanco
        printf("Matriz de rotacion R6 (debe ser igual a R5)\n");
        printf("   r1=%f r2=%f r3=%f\n", p_resultados->R6[0],p_resultados->R6[1],p_resultados->R6[2]);
        printf("   r4=%f r5=%f r6=%f\n", p_resultados->R6[3],p_resultados->R6[4],p_resultados->R6[5]);
        printf("   r7=%f r8=%f r9=%f\n", p_resultados->R6[6],p_resultados->R6[7],p_resultados->R6[8]);
    }
    else {
        printf("\n");
        printf("No se desplegara matriz de rotacion R6\n");
    }

    //Calculando la posicion del punto respecto
    //al sistema de coordenadas local de la cámara
    //en el instante de tiempo k
    H[0]=p_resultados->R[0]*p_parametros->Hokor+p_resultados->R[1]*p_parametros->Hokos+p_resultados->R[2]*p_parametros->Hokot+p_parametros->GokcX;
    H[1]=p_resultados->R[3]*p_parametros->Hokor+p_resultados->R[4]*p_parametros->Hokos+p_resultados->R[5]*p_parametros->Hokot+p_parametros->GokcY;
    H[2]=p_resultados->R[6]*p_parametros->Hokor+p_resultados->R[7]*p_parametros->Hokos+p_resultados->R[8]*p_parametros->Hokot+p_parametros->GokcZ;
    p_resultados->Hokcx=H[0];
    p_resultados->Hokcy=H[1];
    p_resultados->Hokcz=H[2];

    //Calculando la posicion bidimensional del punto con respecto
    //al sistema de coordenadas local de la cámara en el instante de tiempo k
    hk[0]=p_parametros->f*p_resultados->Hokcx/p_resultados->Hokcz;
    hk[1]=p_parametros->f*p_resultados->Hokcy/p_resultados->Hokcz;
    p_resultados->huk=hk[0];
    p_resultados->hvk=hk[1];

    //Calculando La posición bidimensional del punto con respecto al sistema de
    //coordenadas local de la imagen en el instante de tiempo k
    ak[0]=(p_parametros->Nfu/p_parametros->Ncu)*(p_resultados->huk/p_parametros->Du)+(p_parametros->Nfu/2);
    ak[1]=(p_resultados->hvk/p_parametros->Dv)+(p_parametros->Nfv/2);
    p_resultados->cik=ak[0];
    p_resultados->rik=ak[1];

    //Calculando la posicion del punto respecto
    //al sistema de coordenadas del mundo
    //en el instante de tiempo k
    H2[0]=p_resultados->R3[0]*p_resultados->Hokcx+p_resultados->R3[1]*p_resultados->Hokcy+p_resultados->R3[2]*p_resultados->Hokcz+p_parametros->Gckmx;
    H2[1]=p_resultados->R3[3]*p_resultados->Hokcx+p_resultados->R3[4]*p_resultados->Hokcy+p_resultados->R3[5]*p_resultados->Hokcz+p_parametros->Gckmy;
    H2[2]=p_resultados->R3[6]*p_resultados->Hokcx+p_resultados->R3[7]*p_resultados->Hokcy+p_resultados->R3[8]*p_resultados->Hokcz+p_parametros->Gckmz;
    p_resultados->Hokmx=H2[0];
    p_resultados->Hokmy=H2[1];
    p_resultados->Hokmz=H2[2];

    //Calculando la posicion del punto respecto
    //al sistema de coordenadas local de la cámara
    //en el instante de tiempo k+1
    H3[0]=(p_resultados->R5[0]*(p_resultados->Hokcx-p_parametros->GokcX)+p_resultados->R5[1]*(p_resultados->Hokcy-p_parametros->GokcY)+p_resultados->R5[2]*(p_resultados->Hokcz-p_parametros->GokcZ))+p_parametros->GokcX+p_parametros->DTo2cX;
    H3[1]=(p_resultados->R5[3]*(p_resultados->Hokcx-p_parametros->GokcX)+p_resultados->R5[4]*(p_resultados->Hokcy-p_parametros->GokcY)+p_resultados->R5[5]*(p_resultados->Hokcz-p_parametros->GokcZ))+p_parametros->GokcY+p_parametros->DTo2cY;
    H3[2]=(p_resultados->R5[6]*(p_resultados->Hokcx-p_parametros->GokcX)+p_resultados->R5[7]*(p_resultados->Hokcy-p_parametros->GokcY)+p_resultados->R5[8]*(p_resultados->Hokcz-p_parametros->GokcZ))+p_parametros->GokcZ+p_parametros->DTo2cZ;
    p_resultados->Ho2cX=H3[0];
    p_resultados->Ho2cY=H3[1];
    p_resultados->Ho2cZ=H3[2];

    //Calculando la posicion bidimensional del punto con respecto
    //al sistema de coordenadas local de la cámara en el instante de tiempo k+1
    hk2[0]=p_parametros->f*p_resultados->Ho2cX/p_resultados->Ho2cZ;
    hk2[1]=p_parametros->f*p_resultados->Ho2cY/p_resultados->Ho2cZ;
    p_resultados->huk2=hk2[0];
    p_resultados->hvk2=hk2[1];

    //Calculando la posición bidimensional del punto con respecto al sistema de
    //coordenadas local de la imagen en el instante de tiempo k+1
    ak2[0]=(p_parametros->Nfu/p_parametros->Ncu)*(p_resultados->huk2/p_parametros->Du)+(p_parametros->Nfu/2);
    ak2[1]=(p_resultados->hvk2/p_parametros->Dv)+(p_parametros->Nfv/2);
    p_resultados->cik2=ak2[0];
    p_resultados->rik2=ak2[1];


    //Calculando la posicion del objeto con respecto el sistema
    //de coordenadas de la cámara en el instante k+1
    P[0]=(p_parametros->GokcX)+(p_parametros->DTo2cX);
    P[1]=(p_parametros->GokcY)+(p_parametros->DTo2cY);
    P[2]=(p_parametros->GokcZ)+(p_parametros->DTo2cZ);
    p_resultados->Gok2cx=P[0];
    p_resultados->Gok2cy=P[1];
    p_resultados->Gok2cz=P[2]; 

    //Calculando la orientación del objeto con respecto el sistema
    //de coordenadas de la cámara en el instante k+1
    P2[0]=asin(-(p_resultados->R6[6]*p_resultados->R2[0]+p_resultados->R6[7]*p_resultados->R2[3]+p_resultados->R6[8]*p_resultados->R2[6]));
    P2[1]=asin((p_resultados->R6[6]*p_resultados->R2[1]+p_resultados->R6[7]*p_resultados->R2[4]+p_resultados->R6[8]*p_resultados->R2[7])/cos(P2[0]));
    P2[2]=asin((p_resultados->R6[3]*p_resultados->R2[0]+p_resultados->R6[4]*p_resultados->R2[3]+p_resultados->R6[5]*p_resultados->R2[6])/cos(P2[0]));
    p_resultados->Rok2cy=(P2[0])*(180/PI);
    p_resultados->Rok2cx=P2[1]*(180/PI);
    p_resultados->Rok2cz=P2[2]*(180/PI);   

    //Calculando el vector bidimensional que describe el desplazamiento del punto sobre el plano de la cámara
    //entre los instantes de tiempo k y k+1, con respecto al sistema de coordenadas local del plano de la cámara
    D1[0]=p_resultados->huk2-p_resultados->huk;
    D1[1]=p_resultados->hvk2-p_resultados->hvk;
    p_resultados->duk=D1[0];
    p_resultados->dvk=D1[1];

    //Calculando el vector bidimensional que describe el desplazamiento del punto sobre el plano de la cámara
    //entre los instantes de tiempo k y k+1, con respecto al sistema de coordenadas de la imagen
    D2[0]=p_resultados->cik2-p_resultados->cik;
    D2[1]=p_resultados->rik2-p_resultados->rik;
    p_resultados->dck=D2[0];
    p_resultados->drk=D2[1];


    //Desplegando en el terminal los resultados finales
    printf("\n"); //Linea en blanco
    printf("Resultado\n");
    printf("  Posición del punto con respecto al sistema de coordenadas local de la cámara en el instante de tiempo k.\n");
    printf("   Hokcx=%fcm\n", p_resultados->Hokcx);
    printf("   Hokcy=%fcm\n", p_resultados->Hokcy);
    printf("   Hokcz=%fcm\n", p_resultados->Hokcz);
    printf("  Posición bidimensional del punto con respecto al sistema de coordenadas local del plano de la cámara en el instante de tiempo k.\n");
    printf("   huk=%fcm\n", p_resultados->huk);
    printf("   hvk=%fcm\n", p_resultados->hvk);
    printf("  La posición bidimensional del punto con respecto al sistema de coordenadas local de la imagen en el instante de tiempo k.\n");
    printf("   cik=%f píxeles\n", floor(p_resultados->cik));
    printf("   rik=%f píxeles\n", floor(p_resultados->rik));
    printf("  Posición del punto con respecto al sistema de coordenadas del mundo en el instante de tiempo k.\n");
    printf("   Hokmx=%fcm\n", p_resultados->Hokmx);
    printf("   Hokmy=%fcm\n", p_resultados->Hokmy);
    printf("   Hokmz=%fcm\n", p_resultados->Hokmz);
    printf("  Posición del punto con respecto al sistema de coordenadas local de la cámara en el instante de tiempo k+1.\n");
    printf("   Ho2cX=%fcm\n", p_resultados->Ho2cX);
    printf("   Ho2cY=%fcm\n", p_resultados->Ho2cY);
    printf("   Ho2cZ=%fcm\n", p_resultados->Ho2cZ);
    printf("  Posición bidimensional del punto con respecto al sistema de coordenadas local del plano de la cámara en el instante de tiempo k+1.\n");
    printf("   huk2=%fcm\n", p_resultados->huk2);
    printf("   hvk2=%fcm\n", p_resultados->hvk2);
    printf("  La posición bidimensional del punto con respecto al sistema de coordenadas local de la imagen en el instante de tiempo k+1.\n");
    printf("   cik2=%f píxeles\n", floor(p_resultados->cik2));
    printf("   rik2=%f píxeles\n", floor(p_resultados->rik2));
    printf("  Los 6 parámetros que describen la pose del objeto con respecto al sistema de coordenadas de la cámara en el instante de tiempo k+1.\n");
    printf("   Gok2cx=%fcm\n", p_resultados->Gok2cx);
    printf("   Gok2cy=%fcm\n", p_resultados->Gok2cy);
    printf("   Gok2cz=%fcm\n", p_resultados->Gok2cz);
    printf("   Rok2cx=%f grados\n", p_resultados->Rok2cx);
    printf("   Rok2cy=%f grados\n", p_resultados->Rok2cy);
    printf("   Rok2cz=%f grados\n", p_resultados->Rok2cz);
    printf("  Vector bidimensional que describe el desplazamiento del punto sobre el plano de la cámara, entre los instantes de tiempo k y k+1, con respecto al sistema de coordenadas local del plano de la cámara.\n");
    printf("   duk=%fcm\n", p_resultados->duk);
    printf("   dvk=%fcm\n", p_resultados->dvk);
    printf("  Vector bidimensional que describe el desplazamiento del punto sobre el plano de la cámara, entre los instantes de tiempo k y k+1, con respecto al sistema de coordenadas local del plano de la imágen.\n");
    printf("   dck=%f píxeles\n", floor(p_resultados->dck));
    printf("   drk=%f píxeles\n", floor(p_resultados->drk));

    //Salvando los resultados finales en archivo de texto
    danSalvarResultadosEnArchivoDeTexto();

    //Liberando memoria reservada manualmente
    free(p_parametros);
    free(p_resultados->R2);
    free(p_resultados->R4);
    free(p_resultados->R6);
    free(p_resultados);

    return 0;
}
//Fin de programa principal

//Funciones

void danObtenerMatrizDeRotacionR()
{
    double RokcX, RokcY, RokcZ;

    //Conviertiendo angulos a radianes
    RokcX=danConvertirDeGradosARadianes(p_parametros->RokcX);
    RokcY=danConvertirDeGradosARadianes(p_parametros->RokcY);
    RokcZ=danConvertirDeGradosARadianes(p_parametros->RokcZ);

    //Calculando la matriz de rotacion
    p_resultados->R[0]=cos(RokcY)*cos(RokcZ);
    p_resultados->R[1]=sin(RokcX)*sin(RokcY)*cos(RokcZ)-cos(RokcX)*sin(RokcZ);
    p_resultados->R[2]=cos(RokcX)*sin(RokcY)*cos(RokcZ)+sin(RokcX)*sin(RokcZ);
    p_resultados->R[3]=cos(RokcY)*sin(RokcZ);
    p_resultados->R[4]=sin(RokcX)*sin(RokcY)*sin(RokcZ)+cos(RokcX)*cos(RokcZ);
    p_resultados->R[5]=cos(RokcX)*sin(RokcY)*sin(RokcZ)-sin(RokcX)*cos(RokcZ);
    p_resultados->R[6]=-sin(RokcY);
    p_resultados->R[7]=sin(RokcX)*cos(RokcY);
    p_resultados->R[8]=cos(RokcX)*cos(RokcY);
}

void danObtenerMatrizDeRotacionR2(double RokcX, double RokcY, double RokcZ)
{
    //Conviertiendo angulos a radianes
    RokcX=danConvertirDeGradosARadianes(RokcX);
    RokcY=danConvertirDeGradosARadianes(RokcY);
    RokcZ=danConvertirDeGradosARadianes(RokcZ);

    //Calculando la matriz de rotacion
    p_resultados->R2[0]=cos(RokcY)*cos(RokcZ);
    p_resultados->R2[1]=sin(RokcX)*sin(RokcY)*cos(RokcZ)-cos(RokcX)*sin(RokcZ);
    p_resultados->R2[2]=cos(RokcX)*sin(RokcY)*cos(RokcZ)+sin(RokcX)*sin(RokcZ);
    p_resultados->R2[3]=cos(RokcY)*sin(RokcZ);
    p_resultados->R2[4]=sin(RokcX)*sin(RokcY)*sin(RokcZ)+cos(RokcX)*cos(RokcZ);
    p_resultados->R2[5]=cos(RokcX)*sin(RokcY)*sin(RokcZ)-sin(RokcX)*cos(RokcZ);
    p_resultados->R2[6]=-sin(RokcY);
    p_resultados->R2[7]=sin(RokcX)*cos(RokcY);
    p_resultados->R2[8]=cos(RokcX)*cos(RokcY);
}


void danObtenerMatrizDeRotacionR3()
{
    double Rckmx, Rckmy, Rckmz;

    //Conviertiendo angulos a radianes
    Rckmx=danConvertirDeGradosARadianes(p_parametros->Rckmx);
    Rckmy=danConvertirDeGradosARadianes(p_parametros->Rckmy);
    Rckmz=danConvertirDeGradosARadianes(p_parametros->Rckmz);

    //Calculando la matriz de rotacion
    p_resultados->R3[0]=cos(Rckmy)*cos(Rckmz);
    p_resultados->R3[1]=sin(Rckmx)*sin(Rckmy)*cos(Rckmz)-cos(Rckmx)*sin(Rckmz);
    p_resultados->R3[2]=cos(Rckmx)*sin(Rckmy)*cos(Rckmz)+sin(Rckmx)*sin(Rckmz);
    p_resultados->R3[3]=cos(Rckmy)*sin(Rckmz);
    p_resultados->R3[4]=sin(Rckmx)*sin(Rckmy)*sin(Rckmz)+cos(Rckmx)*cos(Rckmz);
    p_resultados->R3[5]=cos(Rckmx)*sin(Rckmy)*sin(Rckmz)-sin(Rckmx)*cos(Rckmz);
    p_resultados->R3[6]=-sin(Rckmy);
    p_resultados->R3[7]=sin(Rckmx)*cos(Rckmy);
    p_resultados->R3[8]=cos(Rckmx)*cos(Rckmy);
}

void danObtenerMatrizDeRotacionR4(double Rckmx, double Rckmy, double Rckmz)
{
    //Conviertiendo angulos a radianes
    Rckmx=danConvertirDeGradosARadianes(Rckmx);
    Rckmy=danConvertirDeGradosARadianes(Rckmy);
    Rckmz=danConvertirDeGradosARadianes(Rckmz);

    //Calculando la matriz de rotacion
    p_resultados->R4[0]=cos(Rckmy)*cos(Rckmz);
    p_resultados->R4[1]=sin(Rckmx)*sin(Rckmy)*cos(Rckmz)-cos(Rckmx)*sin(Rckmz);
    p_resultados->R4[2]=cos(Rckmx)*sin(Rckmy)*cos(Rckmz)+sin(Rckmx)*sin(Rckmz);
    p_resultados->R4[3]=cos(Rckmy)*sin(Rckmz);
    p_resultados->R4[4]=sin(Rckmx)*sin(Rckmy)*sin(Rckmz)+cos(Rckmx)*cos(Rckmz);
    p_resultados->R4[5]=cos(Rckmx)*sin(Rckmy)*sin(Rckmz)-sin(Rckmx)*cos(Rckmz);
    p_resultados->R4[6]=-sin(Rckmy);
    p_resultados->R4[7]=sin(Rckmx)*cos(Rckmy);
    p_resultados->R4[8]=cos(Rckmx)*cos(Rckmy);
}

void danObtenerMatrizDeRotacionR5()
{
    double DRo2cX, DRo2cY, DRo2cZ;

    //Conviertiendo angulos a radianes
    DRo2cX=danConvertirDeGradosARadianes(p_parametros->DRo2cX);
    DRo2cY=danConvertirDeGradosARadianes(p_parametros->DRo2cY);
    DRo2cZ=danConvertirDeGradosARadianes(p_parametros->DRo2cZ);

    //Calculando la matriz de rotacion
    p_resultados->R5[0]=cos(DRo2cY)*cos(DRo2cZ);
    p_resultados->R5[1]=sin(DRo2cX)*sin(DRo2cY)*cos(DRo2cZ)-cos(DRo2cX)*sin(DRo2cZ);
    p_resultados->R5[2]=cos(DRo2cX)*sin(DRo2cY)*cos(DRo2cZ)+sin(DRo2cX)*sin(DRo2cZ);
    p_resultados->R5[3]=cos(DRo2cY)*sin(DRo2cZ);
    p_resultados->R5[4]=sin(DRo2cX)*sin(DRo2cY)*sin(DRo2cZ)+cos(DRo2cX)*cos(DRo2cZ);
    p_resultados->R5[5]=cos(DRo2cX)*sin(DRo2cY)*sin(DRo2cZ)-sin(DRo2cX)*cos(DRo2cZ);
    p_resultados->R5[6]=-sin(DRo2cY);
    p_resultados->R5[7]=sin(DRo2cX)*cos(DRo2cY);
    p_resultados->R5[8]=cos(DRo2cX)*cos(DRo2cY);
}

void danObtenerMatrizDeRotacionR6(double DRo2cX, double DRo2cY, double DRo2cZ)
{
    //Conviertiendo angulos a radianes
    DRo2cX=danConvertirDeGradosARadianes(DRo2cX);
    DRo2cY=danConvertirDeGradosARadianes(DRo2cY);
    DRo2cZ=danConvertirDeGradosARadianes(DRo2cZ);

    //Calculando la matriz de rotacion
    p_resultados->R6[0]=cos(DRo2cY)*cos(DRo2cZ);
    p_resultados->R6[1]=sin(DRo2cX)*sin(DRo2cY)*cos(DRo2cZ)-cos(DRo2cX)*sin(DRo2cZ);
    p_resultados->R6[2]=cos(DRo2cX)*sin(DRo2cY)*cos(DRo2cZ)+sin(DRo2cX)*sin(DRo2cZ);
    p_resultados->R6[3]=cos(DRo2cY)*sin(DRo2cZ);
    p_resultados->R6[4]=sin(DRo2cX)*sin(DRo2cY)*sin(DRo2cZ)+cos(DRo2cX)*cos(DRo2cZ);
    p_resultados->R6[5]=cos(DRo2cX)*sin(DRo2cY)*sin(DRo2cZ)-sin(DRo2cX)*cos(DRo2cZ);
    p_resultados->R6[6]=-sin(DRo2cY);
    p_resultados->R6[7]=sin(DRo2cX)*cos(DRo2cY);
    p_resultados->R6[8]=cos(DRo2cX)*cos(DRo2cY);
}

double danConvertirDeGradosARadianes(double angle)
{
    double res;

    res=angle*PI/180.0;

    return(res);
}

void danLeerParametrosDeControlDeArchivoDeTexto()
{
    FILE *archivo;
    char d1[256], d2[256], d3[256];

    printf("Leyendo los datos de entrada:\n");

    //Abriendo archivo en mode de lectura
    char nombreDeArchivo[256]="current_control_parameters.txt";
    archivo = fopen(nombreDeArchivo, "r");
    if (!archivo) {
        printf("No se pudo abrir el archivo: current_control_parameters.txt\n");
        exit(1);
    }

    //Leyendo datos linea por linea

    //Brincando la primera y segunda lineas
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    fscanf(archivo, "\n");

    printf("  Posicion y orientación de la cámara con respecto al mundo\n");

    //Leyendo Gckmx
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Gckmx=(double)atof(d3);
    printf("   Gckmx: %f\n", p_parametros->Gckmx);
    numeroDeDatosLeidos++;

    //Leyendo Gckmy
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Gckmy=(double)atof(d3);
    printf("   Gckmy: %f\n", p_parametros->Gckmy);
    numeroDeDatosLeidos++;

    //Leyendo Gckmz
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Gckmz=(double)atof(d3);
    printf("   Gckmz: %f\n", p_parametros->Gckmz);
    numeroDeDatosLeidos++;

    //Leyendo Rckmx
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Rckmx=(double)atof(d3);
    printf("   Rckmx: %f\n", p_parametros->Rckmx);
    numeroDeDatosLeidos++;

    //Leyendo Rckmy
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Rckmy=(double)atof(d3);
    printf("   Rckmy: %f\n", p_parametros->Rckmy);
    numeroDeDatosLeidos++;

    //Leyendo Rckmz
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Rckmz=(double)atof(d3);
    printf("   Rckmz: %f\n", p_parametros->Rckmz);
    numeroDeDatosLeidos++;

    //Brincando linea de texto
    fscanf(archivo, "\n");

    printf("  Posición y orientacion del objeto con respecto a la cámara en el instante de tiempo k\n");
    //Leyendo GokcX
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->GokcX=(double)atof(d3);
    printf("   GokcX: %f\n", p_parametros->GokcX);
    numeroDeDatosLeidos++;

    //Leyendo GokcY
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->GokcY=(double)atof(d3);
    printf("   GokcY: %f\n", p_parametros->GokcY);
    numeroDeDatosLeidos++;

    //Leyendo GokcZ
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->GokcZ=(double)atof(d3);
    printf("   GokcZ: %f\n", p_parametros->GokcZ);
    numeroDeDatosLeidos++;

    //Leyendo RokcX
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->RokcX=(double)atof(d3);
    printf("   RokcX: %f\n", p_parametros->RokcX);
    numeroDeDatosLeidos++;

    //Leyendo RokcY
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->RokcY=(double)atof(d3);
    printf("   RokcY: %f\n", p_parametros->RokcY);
    numeroDeDatosLeidos++;

    //Leyendo RokcZ
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->RokcZ=(double)atof(d3);
    printf("   RokcZ: %f\n", p_parametros->RokcZ);
    numeroDeDatosLeidos++;

    //Brincando linea de texto
    fscanf(archivo, "\n");

    printf("  Posicion del punto respecto al objeto\n");

    //Leyendo Hokor
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Hokor=(double)atof(d3);
    printf("   Hokor: %f\n", p_parametros->Hokor);
    numeroDeDatosLeidos++;

    //Leyendo Hokos
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Hokos=(double)atof(d3);
    printf("   Hokos: %f\n", p_parametros->Hokos);
    numeroDeDatosLeidos++;

    //Leyendo Hokot
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Hokot=(double)atof(d3);
    printf("   Hokot: %f\n", p_parametros->Hokot);
    numeroDeDatosLeidos++;

    //Brincando linea de texto
    fscanf(archivo, "\n");

    printf("  Cambio de posición y orientación del objeto con respecto a la cámara entre el instante k y k+1\n");

    //Leyendo DTo2cX
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->DTo2cX=(double)atof(d3);
    printf("   DTo2cX: %f\n", p_parametros->DTo2cX);
    numeroDeDatosLeidos++;

    //Leyendo DTo2cY
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->DTo2cY=(double)atof(d3);
    printf("   DTo2cY: %f\n", p_parametros->DTo2cY);
    numeroDeDatosLeidos++;

    //Leyendo DTo2cZ 
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->DTo2cZ =(double)atof(d3);
    printf("   DTo2cZ : %f\n", p_parametros->DTo2cZ );
    numeroDeDatosLeidos++;

    //Leyendo DRo2cX
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->DRo2cX=(double)atof(d3);
    printf("   DRo2cX: %f\n", p_parametros->DRo2cX);
    numeroDeDatosLeidos++;

    //Leyendo DRo2cY
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->DRo2cY=(double)atof(d3);
    printf("   DRo2cY: %f\n", p_parametros->DRo2cY);
    numeroDeDatosLeidos++;

    //Leyendo DRo2cZ
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->DRo2cZ=(double)atof(d3);
    printf("   DRo2cZ: %f\n", p_parametros->DRo2cZ);
    numeroDeDatosLeidos++;

     //Brincando linea de texto
    fscanf(archivo, "\n");

    printf("  Distancia focal;\n");

    //Leyendo f
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->f=(double)atof(d3);
    printf("   f: %f\n", p_parametros->f);
    numeroDeDatosLeidos++;

      printf("  Píxeles de largo;\n");

    //Leyendo Nfu
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Nfu=(double)atof(d3);
    printf("   Nfu: %f\n", p_parametros->Nfu);
    numeroDeDatosLeidos++;

      printf("  Píxeles de alto;\n");

    //Leyendo Nfv
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Nfv=(double)atof(d3);
    printf("   Nfv: %f\n", p_parametros->Nfv);
    numeroDeDatosLeidos++;

      printf("  Número de sensores por línea;\n");

    //Leyendo Ncu
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Ncu=(double)atof(d3);
    printf("   Ncu: %f\n", p_parametros->Ncu);
    numeroDeDatosLeidos++;

      printf("  Número de sensores por columna;\n");

    //Leyendo Ncv
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Ncv=(double)atof(d3);
    printf("   Ncv: %f\n", p_parametros->Ncv);
    numeroDeDatosLeidos++;

      printf("  Distancia entre sensores a lo largo del eje horizontal;\n");

    //Leyendo Du
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Du=(double)atof(d3);
    printf("   Du: %f\n", p_parametros->Du);
    numeroDeDatosLeidos++;

      printf("  Distancia entre sensores a lo largo del eje vertical;\n");

    //Leyendo Dv
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->Dv=(double)atof(d3);
    printf("   Dv: %f\n", p_parametros->Dv);
    numeroDeDatosLeidos++;

    printf("  Numero de datos leidos: %d\n", numeroDeDatosLeidos);

    //Cerrando archivo
    fclose(archivo);
}

void danSalvarResultadosEnArchivoDeTexto()
{
    FILE *archivo;
    int i;

    //Abriendo archivo en modo de escritura
    char nombreDeArchivo[256]="resultados.txt";
    archivo = fopen(nombreDeArchivo, "w");
    if (!archivo) {
        printf("No se pudo abrir el archivo: resultados.txt\n");
        exit(1);
    }

    //Escribiendo resultados en archivo
    //Guardando matriz de rotacion del objeto con
    //respecto a la cámara en el instante de tiempo k
    if (DESPLIEGUE_MATRIZ_DE_ROTACION==1) {
        fprintf(archivo, "Matriz de rotacion del objeto con respecto a la cámara en el instante de tiempo k\n");
        for (i=0;i<9;i++) {
            fprintf(archivo, "R[%d]=%f\n", i, p_resultados->R[i]);
        }
    }

    //Escribiendo resultados en archivo
    //Guardando matriz de rotacion de la cámara con
    //respecto al mundo en el instante de tiempo k
    if (DESPLIEGUE_MATRIZ_DE_ROTACION==1) {
        fprintf(archivo, "Matriz de rotacion de la cámara con respecto al mundo en el instante de tiempo k\n");
        for (i=0;i<9;i++) {
            fprintf(archivo, "R3[%d]=%f\n", i, p_resultados->R3[i]);
        }
    }

    //Escribiendo resultados en archivo
    //Guardando matriz de rotacion del objeto con
    //respecto a la cámara en el instante de tiempo k+1
    if (DESPLIEGUE_MATRIZ_DE_ROTACION==1) {
        fprintf(archivo, "Matriz de rotacion del objeto con respecto a la cámara en el instante de tiempo k+1\n");
        for (i=0;i<9;i++) {
            fprintf(archivo, "R5[%d]=%f\n", i, p_resultados->R5[i]);
        }
    } 

    fprintf(archivo, "\n"); //linea en blanco
    //Guardando posicion
    fprintf(archivo, "Posición del punto con respecto al sistema de coordenadas local de la cámara en el instante de tiempo k.\n");
    fprintf(archivo, "Hokcx=%fcm\n", p_resultados->Hokcx);
    fprintf(archivo, "Hokcy=%fcm\n", p_resultados->Hokcy);
    fprintf(archivo, "Hokcz=%fcm\n", p_resultados->Hokcz);
    fprintf(archivo, "Posición bidimensional del punto con respecto al sistema de coordenadas local del plano de la cámara en el instante de tiempo k.\n");
    fprintf(archivo, "huk=%fcm\n", p_resultados->huk);
    fprintf(archivo, "hvk=%fcm\n", p_resultados->hvk);
    fprintf(archivo, "La posición bidimensional del punto con respecto al sistema de coordenadas local de la imagen en el instante de tiempo k.\n");
    fprintf(archivo, "cik=%f píxeles\n", floor(p_resultados->cik));
    fprintf(archivo, "rik=%f píxeles\n", floor(p_resultados->rik));
    fprintf(archivo, "Posición del punto con respecto al sistema de coordenadas del mundo en el instante de tiempo k.\n");
    fprintf(archivo, "Hokmx=%fcm\n", p_resultados->Hokmx);
    fprintf(archivo, "Hokmy=%fcm\n", p_resultados->Hokmy);
    fprintf(archivo, "Hokmz=%fcm\n", p_resultados->Hokmz);
    fprintf(archivo, "Posición del punto con respecto al sistema de coordenadas local de la cámara en el instante de tiempo k+1.\n");
    fprintf(archivo, "Ho2cX=%fcm\n", p_resultados->Ho2cX);
    fprintf(archivo, "Ho2cY=%fcm\n", p_resultados->Ho2cY);
    fprintf(archivo, "Ho2cZ=%fcm\n", p_resultados->Ho2cZ);
    fprintf(archivo,  "Posición bidimensional del punto con respecto al sistema de coordenadas local del plano de la cámara en el instante de tiempo k+1.\n");
    fprintf(archivo,  "huk2=%fcm\n", p_resultados->huk2);
    fprintf(archivo,  "hvk2=%fcm\n", p_resultados->hvk2);
    fprintf(archivo, "La posición bidimensional del punto con respecto al sistema de coordenadas local de la imagen en el instante de tiempo k+1.\n");
    fprintf(archivo, "cik2=%f píxeles\n", floor(p_resultados->cik2));
    fprintf(archivo, "rik2=%f píxeles\n", floor(p_resultados->rik2));
    fprintf(archivo,  "Los 6 parámetros que describen la pose del objeto con respecto al sistema de coordenadas de la cámara en el instante de tiempo k+1.\n");
    fprintf(archivo,  "Gok2cx=%fcm\n", p_resultados->Gok2cx);
    fprintf(archivo,  "Gok2cy=%fcm\n", p_resultados->Gok2cy);
    fprintf(archivo,  "Gok2cz=%fcm\n", p_resultados->Gok2cz);
    fprintf(archivo,  "Rok2cx=%f grados\n", p_resultados->Rok2cx);
    fprintf(archivo,  "Rok2cy=%f grados\n", p_resultados->Rok2cy);
    fprintf(archivo,  "Rok2cz=%f grados\n", p_resultados->Rok2cz);
    fprintf(archivo,  "Vector bidimensional que describe el desplazamiento del punto sobre el plano de la cámara, entre los instantes de tiempo k y k+1, con respecto al sistema de coordenadas local del plano de la cámara.\n");
    fprintf(archivo,   "duk=%fcm\n", p_resultados->duk);
    fprintf(archivo,   "dvk=%fcm\n", p_resultados->dvk);
    fprintf(archivo,  "Vector bidimensional que describe el desplazamiento del punto sobre el plano de la cámara, entre los instantes de tiempo k y k+1, con respecto al sistema de coordenadas local del plano de la imágen.\n");
    fprintf(archivo,   "dck=%f píxeles\n", floor(p_resultados->dck));
    fprintf(archivo,   "drk=%f píxeles\n", floor(p_resultados->drk));

    //Cerrando archivo
    fclose(archivo);
}