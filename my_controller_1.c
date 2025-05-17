#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#define TIME_STEP 32
#define VEL_MAX 6.28
#define LIMIAR_ANG 0.05
#define NUM_SENSORES 8
#define NUM_CAIXAS 20
#define LIMIAR_PROXIMIDADE 0.07

typedef struct {
  double x, y, z;
  bool visitada;
} CaixaInfo;

int main() {
  wb_robot_init();

  WbDeviceTag motor_esq = wb_robot_get_device("left wheel motor");
  WbDeviceTag motor_dir = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(motor_esq, INFINITY);
  wb_motor_set_position(motor_dir, INFINITY);

  WbDeviceTag sensores[NUM_SENSORES];
  char nome[5];
  for (int i = 0; i < NUM_SENSORES; i++) {
    sprintf(nome, "ps%d", i);
    sensores[i] = wb_robot_get_device(nome);
    wb_distance_sensor_enable(sensores[i], TIME_STEP);
  }

  WbNodeRef self = wb_supervisor_node_get_self();
  WbFieldRef rot_field = wb_supervisor_node_get_field(self, "rotation");

  CaixaInfo caixas[NUM_CAIXAS];
  for (int i = 0; i < NUM_CAIXAS; i++) {
    char def_nome[20];
    sprintf(def_nome, "CAIXA%02d", i + 1);
    WbNodeRef caixa = wb_supervisor_node_get_from_def(def_nome);
    if (caixa) {
      const double *pos = wb_supervisor_node_get_position(caixa);
      caixas[i].x = pos[0];
      caixas[i].y = pos[1];
      caixas[i].z = pos[2];
      caixas[i].visitada = false;
    } else {
      caixas[i].x = 0;
      caixas[i].y = 0;
      caixas[i].z = 0;
      caixas[i].visitada = true;
    }
  }

  bool desviando = false;
  bool lado_direito = true;
  int caixa_atual = 1;
  double tempo_inicio = wb_robot_get_time();
  WbNodeRef caixa;

  while (wb_robot_step(TIME_STEP) != -1) {
    char def_nome[20];
    sprintf(def_nome, "CAIXA%02d", caixa_atual);
    caixa = wb_supervisor_node_get_from_def(def_nome);

    if (!caixa) {
      printf("Não foi possível encontrar a caixa %d.\n", caixa_atual);
      break;
    }

    const double *pos_robo = wb_supervisor_node_get_position(self);
    const double *rot = wb_supervisor_field_get_sf_rotation(rot_field);

    const CaixaInfo *caixa_destino = &caixas[caixa_atual - 1];
    double dx = caixa_destino->x - pos_robo[0];
    double dy = caixa_destino->y - pos_robo[1];
    double dz = caixa_destino->z - pos_robo[2];
    double distancia = sqrt(dx * dx + dy * dy + dz * dz);

    double ang_obj = atan2(dz, dx);
    double ang_robo = rot[3];
    double diff = ang_obj - ang_robo;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;

    printf("Buscando CAIXA%02d | Distância: %.3f | Ângulo alvo: %.2f | Ângulo robô: %.2f | Diferença: %.2f | Tempo: %.1f s\n",
           caixa_atual, distancia, ang_obj, ang_robo, diff, wb_robot_get_time() - tempo_inicio);

    double tempo_atual = wb_robot_get_time();
    if (tempo_atual - tempo_inicio > 50.0) {
      printf("Tempo excedido para alcançar a caixa %d. Pulando para a próxima.\n", caixa_atual);
      caixa_atual++;
      tempo_inicio = tempo_atual;
      desviando = false;
      wb_motor_set_velocity(motor_esq, 0.0);
      wb_motor_set_velocity(motor_dir, 0.0);
      wb_robot_step(TIME_STEP * 2);
      continue;
    }

    // Verificação de distância para todas as caixas, com base nas posições fixas
    for (int i = 0; i < NUM_CAIXAS; i++) {
        double dx_c = caixas[i].x - pos_robo[0];
        double dy_c = caixas[i].y - pos_robo[1];
        double dz_c = caixas[i].z - pos_robo[2];
        double dist_c = sqrt(dx_c * dx_c + dy_c * dy_c + dz_c * dz_c);
        if (dist_c <= LIMIAR_PROXIMIDADE) {
          printf("Caixa %d alcançada! Distância: %.2f\n", i + 1, dist_c);
          for (int i = 0; i<10000; i++){
            wb_motor_set_velocity(motor_esq, 0.2 * VEL_MAX);
            wb_motor_set_velocity(motor_dir, -0.2 * VEL_MAX);
            wb_robot_step(TIME_STEP * 4);
          }
          if (i + 1 == caixa_atual) {
            caixa_atual++;
            tempo_inicio = wb_robot_get_time();
            desviando = false;
            wb_motor_set_velocity(motor_esq, 0.0);
            wb_motor_set_velocity(motor_dir, 0.0);
            if (caixa_atual > NUM_CAIXAS) {
              printf("O robô completou a tarefa, chegou à última caixa.\n");
              wb_robot_cleanup();
              return 0;
            }
          }
        }
      
    }

    double leituras[NUM_SENSORES];
    for (int i = 0; i < NUM_SENSORES; i++)
      leituras[i] = wb_distance_sensor_get_value(sensores[i]);

    printf("Leituras dos sensores: ");
    for (int i = 0; i < NUM_SENSORES; i++)
      printf("ps%d: %.1f ", i, leituras[i]);
    printf("\n");

    bool obstaculo_frontal = leituras[0] < 70.0 || leituras[1] < 70.0 || leituras[3] < 70.0 ||
                             leituras[4] < 70.0 || leituras[5] < 70.0 || leituras[6] < 70.0 ||
                             leituras[2] < 70.0;

    if (obstaculo_frontal) {
      desviando = true;
      lado_direito = leituras[2] > leituras[6];
    }

    if (desviando) {
      if (obstaculo_frontal) {
        if (lado_direito) {
          wb_motor_set_velocity(motor_esq, -0.2 * VEL_MAX);
          wb_motor_set_velocity(motor_dir, 0.6 * VEL_MAX);
          wb_robot_step(TIME_STEP * 4);
        } else {
          wb_motor_set_velocity(motor_esq, 0.6 * VEL_MAX);
          wb_motor_set_velocity(motor_dir, -0.2 * VEL_MAX);
          wb_robot_step(TIME_STEP * 4);
        }
      } else {
        desviando = false;
      }
    } else {
      if (fabs(diff) > LIMIAR_ANG) {
        double vel = (diff > 0) ? 0.4 : -0.4;
        wb_motor_set_velocity(motor_esq, -vel * VEL_MAX);
        wb_motor_set_velocity(motor_dir, vel * VEL_MAX);
      } else {
        wb_motor_set_velocity(motor_esq, VEL_MAX);
        wb_motor_set_velocity(motor_dir, VEL_MAX);
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}
