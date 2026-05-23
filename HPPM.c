#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define TRUE 1
#define FALSE 0

#define CLAMP_ACOS(x) ((x) > 1.0f ? 1.0f : ((x) < -1.0f ? -1.0f : (x)))

#define EPSILON 0.001f

typedef struct {
    int nc;                 // Número de obstáculos circulares detectados
    float rr;               // Radio de la hiperesfera
    float m1, m2;           // Pendientes 
    int maxsteps;           // Pasos maximos
    float a0, b0;           // Inicio
    float a1, b1;           // Fin
    float **mc;             // Matriz de obstáculos [x, y, radio, k]
    float **tray;           // Matriz de trayectoria
    int pasos_calculados;   // Total de pasos 
} Sistema;

typedef struct {
    float x, y, L;          // Coordenadas actuales y Lambda
    float xa, ya, La;       // Coordenadas y Lambda anteriores (Predictor)
    float C1, C2, C3;       // Centro actual de la hiperesfera de seguimiento
    float rad, r;           // Radios actuales y anteriores de la hiperesfera
    
    float norxa, norya, norLa; // Vector tangente normalizado (dirección del predictor)
    
    float jacob[3][3];      // Matriz Jacobiana 
    float Jinv[3][3];       // Matriz Jacobiana Inversa
    float f[3][1];          // Evaluaciones del sistema de funciones (H1, H2, Esfera)
    float Dd[3][1];         // Punto predictor (Euler)
    float Dd1[3][1];        // Punto corregido ante detección de retroceso (Reversa)
    
    float Q;                // Repulsión en la meta.
    float W_0;              // Repulsión en el inicio.
    int ii;                 // Contador punto actual
    int inr;                // Acumulador de iteraciones de Newton-Raphson
    float m_L1;             // Determinante del paso anterior
} EstadoCalc;

// Funcion de parametro 
float fP(float x, float y) {
    return 1.0f; 
}

// Crear memoria de matriz
float** crear_matriz(int filas, int cols) {
    int i;
    float **m = (float**)malloc(filas * sizeof(float*));
    if (!m) exit(1);
    for(i = 0; i < filas; i++) {
        m[i] = (float*)calloc(cols, sizeof(float));
        if (!m[i]) exit(1);
    }
    return m;
}

// Limpiar memoria de matriz
void liberar_matriz(float **m, int filas) {
    int i;
    if (!m) return;
    for(i = 0; i < filas; i++) {
        free(m[i]);
    }
    free(m);
}

// Utilza regla de cramer / adjunta 
void invertir_matriz_3x3(float J[3][3], float Ji[3][3]) {
    float det = J[0][0]*(J[1][1]*J[2][2] - J[1][2]*J[2][1]) - 
                J[0][1]*(J[1][0]*J[2][2] - J[1][2]*J[2][0]) + 
                J[0][2]*(J[1][0]*J[2][1] - J[1][1]*J[2][0]);

    float invDet = (fabs(det) < 1e-9) ? 0.0f : 1.0f / det;

    Ji[0][0] = (J[1][1]*J[2][2] - J[1][2]*J[2][1]) * invDet;
    Ji[0][1] = -(J[0][1]*J[2][2] - J[0][2]*J[2][1]) * invDet;
    Ji[0][2] = (J[0][1]*J[1][2] - J[0][2]*J[1][1]) * invDet;
    Ji[1][0] = -(J[1][0]*J[2][2] - J[1][2]*J[2][0]) * invDet;
    Ji[1][1] = (J[0][0]*J[2][2] - J[0][2]*J[2][0]) * invDet;
    Ji[1][2] = -(J[0][0]*J[1][2] - J[0][2]*J[1][0]) * invDet;
    Ji[2][0] = (J[1][0]*J[2][1] - J[1][1]*J[2][0]) * invDet;
    Ji[2][1] = -(J[0][0]*J[2][1] - J[0][1]*J[2][0]) * invDet;
    Ji[2][2] = (J[0][0]*J[1][1] - J[0][1]*J[1][0]) * invDet;
}

// Modulo para evaluar y construir jacobiano
void actualizar_jacobiano_y_f(Sistema *s, EstadoCalc *e, int calc_f) {
    float W_px=0, W_py=0, W=0, aux_Wp=0;
    float Obs, raiz_Obs, abs_1, D, dD_dObs;
    int k;

    float fP_val = fP(e->x, e->y);

    // W_px y W_py
    for(k = 0; k < s->nc; k++) {
        Obs = pow(e->x - s->mc[k][0], 2) + pow(e->y - s->mc[k][1], 2) - pow(s->mc[k][2], 2);    // Obs(x,y) = (x-cx)^2 + (y-cy)^2 - r^2

        raiz_Obs = sqrt(pow(Obs, 2) + pow(EPSILON, 2));     // abs_1 = sqrt(Obs^2 + epsilon^2) - epsilon
        abs_1 = raiz_Obs - EPSILON;
        
        D = Obs + abs_1;    // Afuera (Obs > 0) gud. Dentro (Obs < 0) tiende a infinito para hacer objetos solidos
        
        if(fabs(D) < 1e-12f) D = 1e-12f;

        dD_dObs = 1.0f + (Obs / raiz_Obs);  // dD/dObs = 1 + Obs / sqrt(Obs^2 + epsilon^2)

        float P_bar = s->mc[k][3];                          // Regla de la cadena combinada para las derivadas espaciales
        aux_Wp = -(P_bar * fP_val) / pow(D, 2) * dD_dObs;

        W_px += (aux_Wp * (2.0f * (e->x - s->mc[k][0])));   // Sumatoria de las derivadas respecto a X e Y
        W_py += (aux_Wp * (2.0f * (e->y - s->mc[k][1])));
        
        if(calc_f) {
            W += (P_bar * fP_val) / D;
        }
    }

    e->jacob[0][0] = -s->m1;                                                                // Fila 0: Derivadas de H1 (Ecuación de línea base y punto de inicio)
    e->jacob[0][1] = -1.0f; 
    e->jacob[0][2] = -s->b0 - s->m1 * s->a0 + s->m1 * s->a1 + s->b1;

    e->jacob[1][0] = -s->m2 + W_px;                                                         // Fila 1: Derivadas de H2 (Línea meta + Campos de Repulsión)
    e->jacob[1][1] = -1.0f + W_py;
    e->jacob[1][2] = -s->b0 - s->m2*s->a0 + s->b1 + s->m2*s->a1 + e->W_0 - e->Q;
    
    e->jacob[2][0] = 2.0f*e->x - 2.0f*e->C1;                                                // Fila 2: Derivadas de la Hiperesfera de paso
    e->jacob[2][1] = 2.0f*e->y - 2.0f*e->C2;
    e->jacob[2][2] = 2.0f*e->L - 2.0f*e->C3;

    // H(x, y, Lambda) = 0
    if (calc_f) {
        // H1: Controla la progresión homotópica 
        e->f[0][0] = -e->y - s->m1*e->x + s->m1*s->a1 + s->b1 - (1.0f - e->L)*(-s->b0 - s->m1*s->a0 + s->m1*s->a1 + s->b1);
        // H2: Incluye las repulsiones. W (actual) y Q (en la meta, anula el campo en el destino)
        e->f[1][0] = (-e->y - s->m2*e->x + (s->b1 + s->m2*s->a1) + W - e->Q) - (1.0f - e->L)*(-s->b0 - s->m2*s->a0 + (s->b1 + s->m2*s->a1) + e->W_0 - e->Q);
        // S: Restricción esférica que obliga a buscar soluciones a un tamaño de paso
        e->f[2][0] = pow((e->x - e->C1), 2) + pow((e->y - e->C2), 2) + pow((e->L - e->C3), 2) - e->rad*e->rad;
    }
}

void leer_datos(Sistema *s, int bot_id) {
    char arch_pend[64], arch_obs[64];
    sprintf(arch_pend, "pendientes_%d.txt", bot_id);
    sprintf(arch_obs, "obstaculos_%d.txt", bot_id);

    FILE *f1 = fopen(arch_pend, "r");
    float t_a0, t_b0, t_a1, t_b1;
    
    if (f1 == NULL) { printf("Error: No se pudo abrir %s\n", arch_pend); exit(1); }

    fscanf(f1, "%f", &s->rr);
    fscanf(f1, "%f", &s->m1);
    fscanf(f1, "%f", &s->m2);
    fscanf(f1, "%d", &s->maxsteps);
    
    fscanf(f1, "%f", &t_a0);
    fscanf(f1, "%f", &t_b0);
    fscanf(f1, "%f", &t_a1);
    fscanf(f1, "%f", &t_b1);
    
    s->a0 = t_a0;
    s->b0 = t_b0;
    s->a1 = t_a1;
    s->b1 = t_b1;
    fclose(f1);

    FILE *f2 = fopen(arch_obs, "r");
    if (f2 == NULL) { printf("Error: No se pudo abrir %s\n", arch_obs); exit(1); }
    
    s->nc = 0;
    float v0, v1, v2, v3;
    
    while (fscanf(f2, "%f %f %f %f", &v0, &v1, &v2, &v3) == 4) {        // Numero de obstaculos
        s->nc++;
    }
    
    rewind(f2);
    
    s->mc = crear_matriz(s->nc, 4);
    
    for (int k = 0; k < s->nc; k++){
        if (fscanf(f2, "%f %f %f %f", &v0, &v1, &v2, &v3) == 4) {
            s->mc[k][0] = v0;
            s->mc[k][1] = v1;
            s->mc[k][2] = v2;
            s->mc[k][3] = v3;
        } else {
            s->mc[k][0] = 0.0f; s->mc[k][1] = 0.0f; s->mc[k][2] = 0.0f; s->mc[k][3] = 0.0f;
        }
    }
    fclose(f2);

    s->tray = crear_matriz(s->maxsteps, 3);
    
    printf("Bot %d -> Datos leídos: %d círculos detectados automáticamente.\n", bot_id, s->nc);
    printf("Inicio: %f, %f -> Meta: %f, %f\n", s->a0, s->b0, s->a1, s->b1);
}

// Predictor-Corrector
void calcular_trayectoria(Sistema *s) {
    EstadoCalc e; 
    memset(&e, 0, sizeof(EstadoCalc));
    
    e.m_L1 = 1.0f; 
    
    e.ii = 1;
    e.rad = s->rr; 
    e.r = s->rr;

    e.x = s->a0;        // Punto inicial
    e.y = s->b0;
    e.xa = s->a0; 
    e.ya = s->b0;
    
    float pi = 3.141592653589793f;
    float term_L = (s->b1 + (s->m1 * (s->a1 - s->a0)) - s->b0);
    if (fabs(term_L) < 1e-5) term_L = 1e-5f;

    e.La = (e.ya + (s->m1 * e.xa) - (s->b0 + s->m1 * s->a0)) / term_L;      // Lambda Inicial
    e.L = e.La;

    e.C1 = e.xa; e.C2 = e.ya; e.C3 = e.La;      // Inicio de la esfera
    
    float paro = 0.000001f;                 // Tolerancia de error para Newton-Raphson
    int maxiter = 40;                       // Iteraciones máximas por paso de NR
    int signo = (s->m1 > s->m2) ? -1 : 1;   // Dirección de seguimiento de la curva
    int sig_x = (s->a1 > s->a0) ? 1 : -1;
    int sig_y = (s->b1 > s->b0) ? 1 : -1;
    float trayC[3][3] = {0};                // Buffer histórico de últimos 3 centros (usado para detectar retrocesos)
    int cond1 = 0;
    int k;

    s->tray[0][0] = e.xa;
    s->tray[0][1] = e.ya;
    s->tray[0][2] = e.La;

    // Ajuste de los puntos de salida y llegada con solidos 
    float fP_meta = fP(s->a1, s->b1);
    float fP_inicio = fP(s->a0, s->b0);

    for(k = 0; k < s->nc; k++) {
        float P_bar = s->mc[k][3];

        // Cálculo de Q evaluado en punto final (a1, b1)
        float Obs_Q = pow(s->a1 - s->mc[k][0], 2) + pow(s->b1 - s->mc[k][1], 2) - pow(s->mc[k][2], 2);
        float raiz_Q = sqrt(pow(Obs_Q, 2) + pow(EPSILON, 2));
        float D_Q = Obs_Q + (raiz_Q - EPSILON);
        if (fabs(D_Q) < 1e-12f) D_Q = 1e-12f;
        e.Q += (P_bar * fP_meta) / D_Q;

        // Cálculo de W_0 evaluado en punto inicial (a0, b0)
        float Obs_W = pow(s->a0 - s->mc[k][0], 2) + pow(s->b0 - s->mc[k][1], 2) - pow(s->mc[k][2], 2);
        float raiz_W = sqrt(pow(Obs_W, 2) + pow(EPSILON, 2));
        float D_W = Obs_W + (raiz_W - EPSILON);
        if (fabs(D_W) < 1e-12f) D_W = 1e-12f;
        e.W_0 += (P_bar * fP_inicio) / D_W;
    }

    actualizar_jacobiano_y_f(s, &e, FALSE);
    
    // Cofactores para hallar el vector tangente base al inicio de la curva
    float aux1 = e.jacob[0][2]*e.jacob[1][1] - e.jacob[1][2]*e.jacob[0][1];
    float aux2 = e.jacob[1][2]*e.jacob[0][0] - e.jacob[0][2]*e.jacob[1][0];
    float det0 = -(e.jacob[0][0]*e.jacob[1][1] - e.jacob[0][1]*e.jacob[1][0]);
    float norBA0 = sqrt(aux1*aux1 + aux2*aux2 + det0*det0);
    
    if (norBA0 < 1e-9f) norBA0 = 1e-9f;
    
    e.norxa = aux1/norBA0; 
    e.norya = aux2/norBA0; 
    e.norLa = det0/norBA0;

    // Predictor de Euler
    e.Dd[0][0] = e.xa + signo*(s->rr * e.norxa);
    e.Dd[1][0] = e.ya + signo*(s->rr * e.norya);
    e.Dd[2][0] = e.La + signo*(s->rr * e.norLa);
    e.m_L1 = det0; 

    // Homotopia
    while ((e.ii < s->maxsteps-2) && (e.L > -0.5f) && (cond1 == 0)) {
        e.x = e.Dd[0][0]; e.y = e.Dd[1][0]; e.L = e.Dd[2][0];
        
        trayC[0][0] = trayC[1][0]; trayC[0][1] = trayC[1][1]; trayC[0][2] = trayC[1][2];
        trayC[1][0] = trayC[2][0]; trayC[1][1] = trayC[2][1]; trayC[1][2] = trayC[2][2];
        trayC[2][0] = e.C1; trayC[2][1] = e.C2; trayC[2][2] = e.C3;

        s->tray[e.ii][0] = e.xa; 
        s->tray[e.ii][1] = e.ya; 
        s->tray[e.ii][2] = e.La;

        int i = 0; 
        float err = 1.0f;
        while (i < maxiter) {
            actualizar_jacobiano_y_f(s, &e, TRUE);
            err = sqrt(pow(e.f[0][0], 2) + pow(e.f[1][0], 2) + pow(e.f[2][0], 2));
            if (err <= paro) break;

            invertir_matriz_3x3(e.jacob, e.Jinv);
            float a1 = (e.Jinv[0][0]*e.f[0][0] + e.Jinv[0][1]*e.f[1][0] + e.Jinv[0][2]*e.f[2][0]);
            float a2 = (e.Jinv[1][0]*e.f[0][0] + e.Jinv[1][1]*e.f[1][0] + e.Jinv[1][2]*e.f[2][0]);
            float a3 = (e.Jinv[2][0]*e.f[0][0] + e.Jinv[2][1]*e.f[1][0] + e.Jinv[2][2]*e.f[2][0]);
            
            e.x -= a1; 
            e.y -= a2; 
            e.L -= a3;
            e.xa = e.x; 
            e.ya = e.y; 
            e.La = e.L;
            i++;
        }
        e.inr += i;

        // Antireversion 
        if (e.ii > 3) {
            int rever = 1, irot = 1, direction = 1;
            while ((irot < 3) && (rever == 1)) {
                float gr[3]; 
                
                gr[0] = 2*s->tray[e.ii-1][0] - 2*trayC[2][0];
                gr[1] = 2*s->tray[e.ii-1][1] - 2*trayC[2][1];
                gr[2] = 2*s->tray[e.ii-1][2] - 2*trayC[2][2];
                float nor_gr = sqrt(gr[0]*gr[0] + gr[1]*gr[1] + gr[2]*gr[2]);
                if (nor_gr < 1e-9) nor_gr = 1e-9;
                
                float t1[3], t2[3];
                for(int d=0; d<3; d++) t1[d] = floor((acos(CLAMP_ACOS(gr[d]/nor_gr))*(180/pi))*100.0f)/100.0f;
                
                e.x = e.xa; e.y = e.ya; e.L = e.La;
                gr[0] = 2*e.x - 2*trayC[2][0]; gr[1] = 2*e.y - 2*trayC[2][1]; gr[2] = 2*e.L - 2*trayC[2][2];
                nor_gr = sqrt(gr[0]*gr[0] + gr[1]*gr[1] + gr[2]*gr[2]);
                if (nor_gr < 1e-9) nor_gr = 1e-9;

                for(int d=0; d<3; d++) t2[d] = floor((acos(CLAMP_ACOS(gr[d]/nor_gr))*(180/pi))*100.0f)/100.0f;
                
                rever = (t1[0] == t2[0] && t1[1] == t2[1]) ? 1 : 0;
                
                // Forzar rotacion si retrocedio 
                if (err > paro || rever == 1) {
                    float norfi = sqrt(pow(trayC[2][0]-s->tray[e.ii-1][0],2) + pow(trayC[2][1]-s->tray[e.ii-1][1],2) + pow(trayC[2][2]-s->tray[e.ii-1][2],2));
                    if(norfi < 1e-9) norfi = 1e-9;
                    float fi1 = floor((acos(CLAMP_ACOS((trayC[2][0]-s->tray[e.ii-1][0])/norfi))*(180/pi))*100.0f)/100.0f;
                    
                    float norxaux = sqrt(pow(e.Dd[0][0]-trayC[2][0],2) + pow(e.Dd[1][0]-trayC[2][1],2) + pow(e.Dd[2][0]-trayC[2][2],2));
                    if(norxaux < 1e-9) norxaux = 1e-9;
                    
                    actualizar_jacobiano_y_f(s, &e, FALSE); 
                    norxaux = (signo*(e.rad*e.norxa))/norxaux; 
                    
                    // Elegir donde girar
                    if (irot == 1) {
                        float fi2 = floor((acos(CLAMP_ACOS(norxaux))*(180/pi))*100.0f)/100.0f;
                        direction = (fi2 > fi1) ? 1 : -1;
                    } else {
                        direction *= -1;
                    }

                    float ang = (pi/4.0) * direction;
                    float matrot[2][2] = {{cos(ang), -sin(ang)}, {sin(ang), cos(ang)}};
                    float Pnp[2] = {e.Dd[0][0]-trayC[2][0], e.Dd[1][0]-trayC[2][1]};
                    float Prot[2] = {matrot[0][0]*Pnp[0] + matrot[0][1]*Pnp[1], matrot[1][0]*Pnp[0] + matrot[1][1]*Pnp[1]};
                    
                    e.Dd1[0][0] = Prot[0]+trayC[2][0]; 
                    e.Dd1[1][0] = Prot[1]+trayC[2][1];
                    e.Dd1[2][0] = (e.Dd1[1][0]+(s->m1*e.Dd1[0][0])-(s->b0+s->m1*s->a0))/term_L;

                    e.x = e.Dd1[0][0]; e.y = e.Dd1[1][0]; e.L = e.Dd1[2][0];
                    i = 0; 
                    err = 1.0f;
                    
                    while (i < maxiter) {
                        actualizar_jacobiano_y_f(s, &e, TRUE);
                        err = sqrt(pow(e.f[0][0], 2) + pow(e.f[1][0], 2) + pow(e.f[2][0], 2));
                        if (err <= paro) break;

                        invertir_matriz_3x3(e.jacob, e.Jinv);
                        float ra1=(e.Jinv[0][0]*e.f[0][0]+e.Jinv[0][1]*e.f[1][0]+e.Jinv[0][2]*e.f[2][0]);
                        float ra2=(e.Jinv[1][0]*e.f[0][0]+e.Jinv[1][1]*e.f[1][0]+e.Jinv[1][2]*e.f[2][0]);
                        float ra3=(e.Jinv[2][0]*e.f[0][0]+e.Jinv[2][1]*e.f[1][0]+e.Jinv[2][2]*e.f[2][0]);
                        
                        e.x -= ra1; 
                        e.y -= ra2; 
                        e.L -= ra3; 
                        e.xa = e.x; 
                        e.ya = e.y; 
                        e.La = e.L;
                        i++;
                    }
                    e.inr += i;
                }
                irot++; 
            }
        }

        // Modular pasos
        actualizar_jacobiano_y_f(s, &e, FALSE);
        float det = -1.0f*(e.jacob[0][0]*e.jacob[1][1] - e.jacob[0][1]*e.jacob[1][0]);
        float den = e.jacob[0][0]*e.jacob[1][1] - e.jacob[0][1]*e.jacob[1][0];
        if(fabs(den) < 1e-9) den = 1e-9f; 

        float aux_Dd1 = (-det*e.jacob[0][2])*(e.jacob[1][1]/den) + (-det*e.jacob[1][2])*(-e.jacob[0][1]/den);
        float aux_Dd2 = (-det*e.jacob[0][2])*(-e.jacob[1][0]/den) + (-det*e.jacob[1][2])*(e.jacob[0][0]/den);
        float aux_Dd3 = det;
        float norB_A = sqrt(pow(aux_Dd1, 2) + pow(aux_Dd2, 2) + pow(aux_Dd3, 2));
        if(norB_A < 1e-9) norB_A = 1e-9f;
        
        e.norxa = aux_Dd1 / norB_A; 
        e.norya = aux_Dd2 / norB_A; 
        e.norLa = aux_Dd3 / norB_A;
        
        float m_L2 = det;
        float exp_term = exp(-fabs(e.m_L1) / sqrt(pow(m_L2, 2) + 1e-9f));
        e.rad = s->rr * (1.0f + exp_term); 
        e.m_L1 = m_L2;
        
        // No dar pasos mas grandes del permitido 
        if ((e.r - e.rad) > (e.r / 3.0f)) {
            e.rad = (e.r / 2.0f) + s->rr;
        }
        
        float norBA = sqrt(pow(e.xa - s->tray[e.ii][0], 2) + pow(e.ya - s->tray[e.ii][1], 2) + pow(e.La - s->tray[e.ii][2], 2));
        if(fabs(norBA) < 1e-9) norBA = 1e-9f;

        float factor = (e.rad - e.r) / norBA;
        e.C1 = e.xa + factor*(e.xa - s->tray[e.ii][0]);
        e.C2 = e.ya + factor*(e.ya - s->tray[e.ii][1]);
        e.C3 = e.La + factor*(e.La - s->tray[e.ii][2]);
        
        e.Dd[0][0] = e.C1 + signo*(e.rad*e.norxa);
        e.Dd[1][0] = e.C2 + signo*(e.rad*e.norya);
        e.Dd[2][0] = e.C3 + signo*(e.rad*e.norLa);

        int lim_x = (sig_x==1) ? (e.xa < s->a1-0.01f ? 0 : 1) : (e.xa > s->a1+0.01f ? 0 : 1);
        int lim_y = (sig_y==1) ? (e.ya < s->b1-0.01f ? 0 : 1) : (e.ya > s->b1+0.01f ? 0 : 1);
        cond1 = (e.La >= 1 && lim_x && lim_y) ? 1 : 0;

        e.r = e.rad;
        e.ii++;
    }

    s->tray[e.ii][0] = e.xa;
    s->tray[e.ii][1] = e.ya;
    s->tray[e.ii][2] = e.La;
    
    s->pasos_calculados = e.ii + 1; 

    printf("Pasos totales: %d\n", e.ii+1);
}  

void guardar_resultados(Sistema *s, int bot_id) {
    char arch_tray[64];
    sprintf(arch_tray, "tray_%d.txt", bot_id);

    FILE *fp2 = fopen(arch_tray, "w");
    int i;
    if (fp2 == NULL) {
        printf("Error: No se pudo crear %s\n", arch_tray);
        return;
    }

    for (i = 0; i < s->pasos_calculados; i++){
        fprintf(fp2, "%f \t %f \n", s->tray[i][0], s->tray[i][1]);
    }
    fclose(fp2);
    printf("%s generado exitosamente.\n", arch_tray);
}

int main(int argc, char *argv[]) {
    int bot_id = 1;
    if (argc > 1) {
        bot_id = atoi(argv[1]);
    }

    Sistema s;
    memset(&s, 0, sizeof(Sistema));

    leer_datos(&s, bot_id);
    calcular_trayectoria(&s); 
    guardar_resultados(&s, bot_id);
    
    liberar_matriz(s.mc, s.nc);
    liberar_matriz(s.tray, s.maxsteps);

    return 0;
}