// test_zaic.cpp
// 使用 BHV_ConstantDepth 生成 IvP 函数，并将 depth-utility 曲线导出为 CSV

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

#include "BHV_ConstantDepth.h"
#include "BuildUtils.h"
#include "InfoBuffer.h"
#include "IvPBox.h"
#include "PDMap.h"

using namespace std;

// 在给定深度索引上，从 PDMap 中采样效用，covered=true 表示该点在 IvP 函数定义域内
double sampleDepthUtility(PDMap* pdmap, unsigned int depth_index, bool& covered)
{
  covered = false;

  if (!pdmap || pdmap->getDim() != 1)
    return 0.0;

  IvPDomain domain = pdmap->getDomain();
  unsigned int depth_points = domain.getVarPoints(0);
  if (depth_points == 0 || depth_index >= depth_points)
    return 0.0;

  IvPBox sample_box(pdmap->getDim(), pdmap->getDegree());
  sample_box.setPTS(0, depth_index, depth_index);  // 第 0 维一个单点

  return pdmap->evalPoint(&sample_box, &covered);
}

int main()
{
  // 1) 构造 1D IvPDomain：depth ∈ [0,599]，共有 600 个离散点
  IvPDomain domain = stringToDomain("depth,0,599,600");

  // 2) 创建 ConstantDepth 行为
  BHV_ConstantDepth bhv(domain);

  // 3) 创建 InfoBuffer 并挂到行为上（模拟来自 helm 的信息）
  InfoBuffer ibuf;
  ibuf.setCurrTime(0.0);
  bhv.setInfoBuffer(&ibuf);

  // =============================
  // 4) 设置行为参数（ZAIC_PEAK 参数）
  // summit=250, peakwidth=85, basewidth=70, summitdelta=40
  // maxutil/minutil 由 ZAIC_PEAK 内部默认值决定（BHV_ConstantDepth 未暴露）
  // =============================

  const double desired_depth = 250.0;

  bhv.setParam("depth",       "250");  // summit
  bhv.setParam("peakwidth",   "85");
  bhv.setParam("basewidth",   "70");
  bhv.setParam("summitdelta", "40");
  bhv.setParam("maxutil", "250");
  bhv.setParam("minutil", "25");

  // 设置 ownship 当前深度（updateInfoIn 里会用 NAV_DEPTH）
  ibuf.setValue("NAV_DEPTH", desired_depth);

  // 5) 调用 onRunState() 生成 IvPFunction
  IvPFunction* ipf = bhv.onRunState();
  if (!ipf) {
    cerr << "Error: Behavior did not generate an IvP function." << endl;
    return 1;
  }

  PDMap* pdmap = ipf->getPDMap();
  if (!pdmap) {
    cerr << "Error: IvPFunction has no PDMap attached." << endl;
    delete ipf;
    return 1;
  }

  cout << fixed << setprecision(2);
  cout << "Priority weight: " << ipf->getPWT() << '\n';
  cout << "PDMap pieces: " << pdmap->size()
       << ", degree: " << pdmap->getDegree() << '\n';

  IvPDomain pd_domain = pdmap->getDomain();
  for (unsigned int i = 0; i < pd_domain.size(); ++i) {
    cout << "  - " << pd_domain.getVarName(i) << " in ["
         << pd_domain.getVarLow(i) << ", " << pd_domain.getVarHigh(i)
         << "] with " << pd_domain.getVarPoints(i) << " points\n";
  }

  // 6) 打开 CSV 文件（当前工作目录）
  const std::string csv_name = "zaic_constant_depth.csv";
  ofstream ofs(csv_name.c_str());
  if (!ofs.is_open()) {
    cerr << "Error: Could not open " << csv_name << " for writing." << endl;
    delete ipf;
    return 1;
  }

  // 写表头
  ofs << "depth,utility\n";

  // 7) 扫描整个 depth 轴：遍历 IvP domain 中所有离散点
  unsigned int depth_points = pd_domain.getVarPoints(0);
  for (unsigned int idx = 0; idx < depth_points; ++idx) {
    double depth = pd_domain.getVal(0, idx);
    bool covered = false;
    double util  = sampleDepthUtility(pdmap, idx, covered);
    if (!covered)
      continue;

    ofs << depth << "," << util << "\n";
  }

  ofs.close();
  cout << "Wrote depth-utility table to ./" << csv_name << endl;

  // 8) 清理
  delete ipf;
  return 0;
}
