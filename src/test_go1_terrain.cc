#include <chrono>
#include <cstring>
#include <thread>

#include "data/go1_recorded_control.h"
#include "mujoco/mujoco.h"
#include "spdlog/spdlog.h"

int main(int argc, char **argv) {
  char error[1000] = "Could not load binary model";
  mjModel *m       = mj_loadXML(
      "/home/breakds/syncthing/workspace/incoming/go1_terrain/scene.xml", 0, error, 1000);
  mjData *d = mj_makeData(m);

  int testkey = mj_name2id(m, mjOBJ_KEY, "test");

  if (testkey >= 0) {
    mju_copy(d->qpos, m->key_qpos + testkey * m->nq, m->nq);
    mju_copy(d->qvel, m->key_qvel + testkey * m->nv, m->nv);
    mju_copy(d->act, m->key_act + testkey * m->na, m->na);
  }

  std::vector<mjtNum> ctrl = mpg::GetRecordedControls();

  size_t contacts    = 0;
  size_t constraints = 0;

  size_t nsteps = ctrl.size() / m->nu;

  auto start = std::chrono::system_clock::now();

  for (size_t i = 0; i < nsteps; ++i) {
    mju_copy(d->ctrl, ctrl.data() + i * m->nu, m->nu);
    mj_step(m, d);
    contacts += d->ncon;
    constraints += d->nefc;
  }
  std::chrono::duration<double> simtime = std::chrono::system_clock::now() - start;

  spdlog::info("Time elapsed: {}", simtime.count());
  spdlog::info("nsteps = {}", nsteps);
  spdlog::info(" Realtime factor       : {} x\n",
               nsteps * m->opt.timestep / simtime.count());
  spdlog::info("Steps per second       : {}", nsteps / simtime.count());
  spdlog::info("Contacts per step      : {}", contacts / static_cast<double>(nsteps));
  spdlog::info("Constraints per step   : {}", constraints / static_cast<double>(nsteps));

  return 0;
}
