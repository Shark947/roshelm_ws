#include <iostream>
#include <string>

#include "HelmIvP.h"

int main(int argc, char *argv[])
{
  std::string config_path = "src/helm/config/startHelm.yaml";
  if (argc > 1)
    config_path = argv[1];

  HelmIvP helm;
  helm.setConfigPath(config_path);

  if (!helm.OnStartUp())
    return 1;

  helm.Run();
  return 0;
}

