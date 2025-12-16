#include <iomanip>
#include <iostream>
#include <vector>
#include <cmath>

#include "BHV_ConstantHeading.h"
#include "BuildUtils.h"
#include "InfoBuffer.h"
#include "IvPBox.h"
#include "PDMap.h"

namespace {

struct SimInput {
  double time;
  double heading;
};

double wrap360(double value) {
  double wrapped = std::fmod(value, 360.0);
  if (wrapped < 0)
    wrapped += 360.0;
  return wrapped;
}

double sampleHeadingUtility(PDMap *pdmap, double heading) {
  if (!pdmap || pdmap->getDim() != 1)
    return 0.0;

  IvPDomain domain = pdmap->getDomain();
  unsigned int index = domain.getDiscreteVal(0, heading, 2);

  IvPBox sample_box(pdmap->getDim(), pdmap->getDegree());
  sample_box.setPTS(0, index, index);
  sample_box.setBDS(0, true, true);

  return pdmap->evalPoint(&sample_box);
}

void printIvPFunctionDetails(IvPFunction *ipf,
                             const std::vector<double> &sample_headings) {
  if (!ipf)
    return;

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  Priority weight: " << ipf->getPWT() << '\n';

  PDMap *pdmap = ipf->getPDMap();
  if (!pdmap) {
    std::cout << "  (No PDMap attached)" << std::endl;
    return;
  }

  IvPDomain pd_domain = pdmap->getDomain();
  std::cout << "  PDMap pieces: " << pdmap->size()
            << ", degree: " << pdmap->getDegree() << '\n';
  std::cout << "  Variable info:" << std::endl;
  for (unsigned int i = 0; i < pd_domain.size(); ++i) {
    std::cout << "    - " << pd_domain.getVarName(i) << " in ["
              << pd_domain.getVarLow(i) << ", " << pd_domain.getVarHigh(i)
              << "] with " << pd_domain.getVarPoints(i) << " points" << '\n';
  }
  std::cout << "  Grid config: " << pdmap->getGridConfig() << '\n';
  std::cout << "  Utility range: [" << pdmap->getMinWT() << ", "
            << pdmap->getMaxWT() << "]" << '\n';

  for (double heading : sample_headings) {
    double wrapped_heading = wrap360(heading);
    double util = sampleHeadingUtility(pdmap, wrapped_heading);
    std::cout << "    Utility at " << std::setw(6) << wrapped_heading
              << " deg: " << util << '\n';
  }
}

} // namespace

int main() {
  IvPDomain domain = stringToDomain("course,0,359,360");
  BHV_ConstantHeading bhv(domain);

  InfoBuffer buffer;
  buffer.setCurrTime(0.0);
  bhv.setInfoBuffer(&buffer);

  const double desired_heading = 90.0;
  bhv.setParam("heading", std::to_string(desired_heading));
  bhv.setParam("peakwidth", "25");
  bhv.setParam("basewidth", "180");
  bhv.setParam("summitdelta", "20");
  bhv.setParam("priority", "150");

  std::vector<SimInput> inputs = {
      {0.0, 45.0}, {5.0, 60.0}, {10.0, 110.0}, {15.0, 175.0}};

  for (const auto &input : inputs) {
    buffer.setCurrTime(input.time);
    buffer.setValue("NAV_HEADING", input.heading);

    std::cout << "\n--- Simulation step @ t=" << input.time
              << "s, NAV_HEADING=" << input.heading << " deg ---" << std::endl;

    if (IvPFunction *ipf = bhv.onRunState()) {
      std::vector<double> sample_headings = {
          input.heading, desired_heading, desired_heading - 30.0,
          desired_heading + 30.0};
      printIvPFunctionDetails(ipf, sample_headings);
      delete ipf;
    } else {
      std::cout << "  (Behavior did not generate an IvP function)" << std::endl;
    }
  }

  return 0;
}
