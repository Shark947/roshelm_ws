#include <algorithm>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <unordered_map>
#include <vector>

#include "HelmCheck.h"
#include "Notify.h"

int main()
{
  constexpr double pi = 3.14159265358979323846;
  constexpr double radians_per_degree = pi / 180.0;
  constexpr double dt = 0.25; // seconds

  // --- log directory ---
  const std::string log_dir = "log";
  const std::string config_path = "src/helm/config/startHelm.yaml";
  HelmCheck helm;
  helm.setConfigPath(config_path);

  const int dir_status = mkdir(log_dir.c_str(), 0755);
  if (dir_status != 0 && errno != EEXIST)
  {
    std::perror("mkdir");
    return 1;
  }

  const std::vector<std::string> log_keys = {
      "NAV_X", "NAV_Y", "NAV_HEADING", "NAV_DEPTH",
      "NAV_SPEED", "DESIRED_HEADING", "DESIRED_SPEED", "DESIRED_DEPTH"};

  std::unordered_map<std::string, std::ofstream> log_streams;
  for (const auto &key : log_keys)
  {
    log_streams[key].open(log_dir + "/" + key + ".txt", std::ios::trunc);
  }

  auto log_value = [&log_streams](const std::string &name, double value,
                                  double timestamp) {
    auto it = log_streams.find(name);
    if (it != log_streams.end() && it->second.is_open())
    {
      it->second << name << ' ' << std::fixed << std::setprecision(3) << value
                 << ' ' << timestamp << std::endl;
    }
  };

  // --- startup ---
  const bool startup_ok = helm.OnStartUp();
  std::cout << "[startup] ok=" << std::boolalpha << startup_ok
            << " status=" << helm.getStatus()
            << " control=" << helm.hasControl() << std::noboolalpha << std::endl;

  std::cout << "[startup] params:";
  for (const auto &param : helm.getStartupParams())
    std::cout << ' ' << param;
  std::cout << std::endl;

  // Install mail handler for Notify()
  SetNotifyHandler([&](std::list<HelmMsg> &mail) { return helm.OnNewMail(mail); });

  Notify("MOOS_MANUAL_OVERRIDE", std::string("false"));
  Notify("MOOS_MANUAL_OVERIDE", std::string("false"));

  // Mission control flags (posted once - Helm behaviors may update them)
  Notify("DEPLOY", std::string("true"), /*dfTime=*/0.0);
  Notify("RETURN", std::string("false"), /*dfTime=*/0.0);

  const bool const_speed_requested = (std::getenv("ROSHELM_ENABLE_CONST_SPD") != nullptr);
  if (const_speed_requested)
  {
    Notify("SPD", std::string("true"), /*dfTime=*/0.0);
  }

  // --- Desired targets (populated from Helm outputs) ---
  std::unordered_map<std::string, double> desired_targets = {
      {"DESIRED_HEADING", 0.0},
      {"DESIRED_SPEED", 0.0},
      {"DESIRED_DEPTH", 0.0},
  };

  // --- nav state (match startHelm.yaml initial_values) ---
  double nav_x = 0.0;
  double nav_y = 0.0;
  double nav_heading = 0.0;
  double nav_speed = 0.0;
  double nav_depth = 0.0;

  auto wrap_heading = [](double heading) {
    double wrapped = std::fmod(heading, 360.0);
    if (wrapped < 0.0)
      wrapped += 360.0;
    return wrapped;
  };

  // Run helm startup checks
  if (!runHelmCheck(helm, startup_ok, std::cout))
  {
    std::cerr << "HelmIvP startup checks failed" << std::endl;
    return 1;
  }

  // -------------------------------------------------------
  //               MAIN SIM LOOP
  // -------------------------------------------------------
  for (int i = 0; i < 500; ++i)
  {
    double t = static_cast<double>(i) * dt;

    if (i == 200)
    {
      Notify("RETURN", std::string("true"), t);
      std::cout << "[mail] t=" << t << " RETURN=true (commanded)" << std::endl;
    }

    // --- publish NAV_* ---
    std::vector<std::pair<std::string, double>> nav_frame = {
        {"NAV_X", nav_x}, {"NAV_Y", nav_y},
        {"NAV_HEADING", nav_heading}, {"NAV_DEPTH", nav_depth},
        {"NAV_SPEED", nav_speed}};

    for (const auto &datum : nav_frame)
    {
      Notify(datum.first, datum.second, t);
      std::cout << "[mail] t=" << t << " " << datum.first << "=" << datum.second << std::endl;
      log_value(datum.first, datum.second, t);
    }

    // --- Helm iteration ---
    helm.Iterate();
    helm.printReport(std::cout);

    // --- Retrieve desired values from Helm behaviors ---
    auto desired = helm.collectDesiredDoubles();
    if (!desired.empty())
    {
      std::cout << "[desired] iter=" << i << " ";
      for (const auto &entry : desired)
      {
        desired_targets[entry.first] = entry.second;
        std::cout << entry.first << "=" << entry.second << " ";
        log_value(entry.first, entry.second, t);
      }
      std::cout << std::endl;
    }
    else
    {
      std::cout << "[desired] iter=" << i << " (none)" << std::endl;
    }

    std::cout << "[status] helm=" << helm.getStatus()
              << " control=" << helm.hasControl() << std::endl;

    // =====================================================
    // 方案 A：直接使用 Helm 的输出控制运动学
    // =====================================================
    double helm_heading = wrap_heading(desired_targets["DESIRED_HEADING"]);
    double helm_speed   = desired_targets["DESIRED_SPEED"];
    double helm_depth   = desired_targets["DESIRED_DEPTH"];

    nav_heading = helm_heading;
    nav_speed   = std::max(0.0, helm_speed);
    nav_depth   = std::max(0.0, helm_depth);

    // --- Kinematics update (applied after reading Helm outputs) ---
    // MOOS heading is 0deg=N, 90deg=E, so use sin for X (east) and cos for
    // Y (north) to keep the position integration aligned with the helm frame.
    nav_x += nav_speed * std::sin(nav_heading * radians_per_degree) * dt;
    nav_y += nav_speed * std::cos(nav_heading * radians_per_degree) * dt;

    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
  }

  std::cout << "Simulation complete" << std::endl;
  return 0;
}
