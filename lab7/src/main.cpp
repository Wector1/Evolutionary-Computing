#include "problem/TSPAInstance.h"
#include "solvers/ILS.h"
#include "solvers/LNS.h"
#include "solvers/LocalGreedy.h"
#include "solvers/LocalSteepest.h"
#include "solvers/MSLS.h"
#include "solvers/NearestNeighbour.h"
#include "solvers/NearestNeighbour2Regret.h"
#include "solvers/NearestNeighbour2RegretWeighted.h"
#include "solvers/NearestNeighbourAny.h"
#include "solvers/NearestNeighbourCycle.h"
#include "solvers/NeighbourSteepest.h"
#include "solvers/PrevDeltas.h"
#include "solvers/RandomSearch.h"
#include "solvers/kRegret.h"
#include "solvers/kRegretCycle.h"
#include "solvers/kRegretCycleWeighted.h"
#include "utils/SolverRunner.h"
#include "utils/TSPVisualizer.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

std::string multipleCharacter(char c, int multiplier) {
  if (multiplier < 0)
    return "";
  std::string s = "";
  for (int i = 0; i < multiplier; i++) {
    s += c;
  }
  return s;
}

void printAndVisualizeBetter(
    std::vector<std::pair<SolverStatistics, SolverStatistics>> stats,
    std::vector<std::string> names, bool enableVisualization,
    TSPAInstance tspaInstanceA, TSPAInstance tspaInstanceB) {

  int maxNameLength = 0;
  for (auto &name : names) {
    maxNameLength = std::max(maxNameLength, (int)name.size());
  }
  maxNameLength += 2;

  std::cout << "### Results of experiments, given in format \"avg (min, max)\""
            << std::endl
            << std::endl;
  std::cout << "#### Objective function" << std::endl << std::endl;
  std::cout << "| " << "Method " << std::setw(maxNameLength) << "| TSPA A"
            << multipleCharacter(' ', 17) << "| TSPA B"
            << multipleCharacter(' ', 17) << "|" << std::endl;
  std::cout << "| :" << multipleCharacter('-', maxNameLength - 4) << ": "
            << "| " << multipleCharacter('-', 24 - 2) << " "
            << "| " << multipleCharacter('-', 24 - 2) << " |" << std::endl;
  for (int i = 0; i < stats.size(); i++) {
    std::cout << "| " << names[i]
              << multipleCharacter(' ', maxNameLength - names[i].size() - 2)
              << " | " << stats[i].first.averageCost << " ("
              << stats[i].first.minCost << ", " << stats[i].first.maxCost
              << ") | " << stats[i].second.averageCost << " ("
              << stats[i].second.minCost << ", " << stats[i].second.maxCost
              << ") |" << std::endl;
  }
  std::cout << std::endl
            << "#### Computation times [ms]" << std::endl
            << std::endl;
  std::cout << "| Method" << multipleCharacter(' ', maxNameLength - 7)
            << "| TSPA A" << multipleCharacter(' ', 20) << "| TSPA B"
            << multipleCharacter(' ', 20) << "|" << std::endl;
  std::cout << "| :" << multipleCharacter('-', maxNameLength - 4) << ": "
            << "| " << multipleCharacter('-', 27 - 2) << " "
            << "| " << multipleCharacter('-', 27 - 2) << " |" << std::endl;
  for (int i = 0; i < stats.size(); i++) {
    std::cout << "| " << names[i]
              << multipleCharacter(' ', maxNameLength - names[i].size() - 2)
              << " | " << stats[i].first.averageRuntimeMs << " ("
              << stats[i].first.minRuntimeMs << ", "
              << stats[i].first.maxRuntimeMs << ") | "
              << stats[i].second.averageRuntimeMs << " ("
              << stats[i].second.minRuntimeMs << ", "
              << stats[i].second.maxRuntimeMs << ") |" << std::endl;
  }

  std::cout << std::endl << "#### Steps" << std::endl << std::endl;
  std::cout << "| Method" << multipleCharacter(' ', maxNameLength - 7)
            << "| TSPA A" << multipleCharacter(' ', 20) << "| TSPA B"
            << multipleCharacter(' ', 20) << "|" << std::endl;
  std::cout << "| :" << multipleCharacter('-', maxNameLength - 4) << ": "
            << "| " << multipleCharacter('-', 27 - 2) << " "
            << "| " << multipleCharacter('-', 27 - 2) << " |" << std::endl;
  for (int i = 0; i < stats.size(); i++) {
    std::cout << "| " << names[i]
              << multipleCharacter(' ', maxNameLength - names[i].size() - 2)
              << " | " << stats[i].first.averageSteps << " ("
              << stats[i].first.minSteps << ", " << stats[i].first.maxSteps
              << ") | " << stats[i].second.averageSteps << " ("
              << stats[i].second.minSteps << ", " << stats[i].second.maxSteps
              << ") |" << std::endl;
  }

  std::cout << "\n### 2D visualization of the best solution for each instance "
               "and method. Cost of nodes should be presented e.g. by a color, "
               "greyscale, or size."
            << std::endl
            << std::endl;
  std::cout << "#### TSP A:" << std::endl << std::endl;
  for (int i = 0; i < stats.size(); i++) {
    std::string cleanName = names[i];
    std::replace(cleanName.begin(), cleanName.end(), ' ', '_');
    if (enableVisualization) {
      std::string outputFile =
          "solutions/images/" + cleanName + "_TSP_A" + ".png";
      TSPVisualizer::visualize(stats[i].first.bestSolution, tspaInstanceA,
                               cleanName + " - " + "TSP_A", outputFile);
    }
    std::cout << i + 1 << ". " << names[i] << ": ![[" << cleanName
              << "_TSPA.png]]" << std::endl;
  }
  std::cout << std::endl << "#### TSP B:" << std::endl << std::endl;
  for (int i = 0; i < stats.size(); i++) {
    std::string cleanName = names[i];
    std::replace(cleanName.begin(), cleanName.end(), ' ', '_');
    if (enableVisualization) {
      std::string outputFile =
          "solutions/images/" + cleanName + "_TSP_B" + ".png";
      TSPVisualizer::visualize(stats[i].second.bestSolution, tspaInstanceB,
                               cleanName + " - " + "TSP_B", outputFile);
    }
    std::cout << i + 1 << ". " << names[i] << ": ![[" << cleanName
              << "_TSPB.png]]" << std::endl;
  }

  std::cout << "\n### The best solutions for each instance and method "
               "presented as a list of nodes indices (starting from 0)."
            << std::endl
            << std::endl;

  for (int i = 0; i < stats.size(); i++) {
    std::cout << "\n#### " << names[i] << std::endl << std::endl;
    std::cout << "**TSP_A**" << std::endl;
    std::cout << "```" << std::endl;
    for (auto &it : stats[i].first.bestSolution.getSelectedNodes()) {
      std::cout << it << " ";
    }
    std::cout << "\n```" << std::endl << std::endl;

    std::cout << "**TSP_B**" << std::endl;
    std::cout << "```" << std::endl;
    for (auto &it : stats[i].second.bestSolution.getSelectedNodes()) {
      std::cout << it << " ";
    }
    std::cout << "\n```" << std::endl;
  }
}
// std::cout << "=== " << solverName << " ===" << std::endl;
// std::cout << "Total execution time (ms): " << stats.totalRuntimeMs
//           << std::endl;
// std::cout << "Min run time (ms): " << stats.minRuntimeMs << std::endl;
// std::cout << "Max run time (ms): " << stats.maxRuntimeMs << std::endl;
// std::cout << "Average run time (ms): " << stats.averageRuntimeMs <<
// std::endl; std::cout << "Number of runs: " << stats.numRuns << std::endl;
// std::cout << "Best cost: " << stats.minCost << std::endl;
// std::cout << "Worst cost: " << stats.maxCost << std::endl;
// std::cout << "Average cost: " << stats.averageCost << std::endl;
// std::cout << "Best path length: " << stats.bestSolution.getPathLength()
//           << std::endl;
// std::cout << "Best path: " << std::endl;
// std::cout << solverName << " | " << stats.averageCost << " (" <<
// stats.minCost
//           << ", " << stats.maxCost << ")" << std::endl;
// std::cout << solverName << " | " << stats.averageRuntimeMs << " ("
//           << stats.minRuntimeMs << ", " << stats.maxRuntimeMs << ")"
//           << std::endl;
// for (int node : stats.bestSolution.getSelectedNodes()) {
//   std::cout << node << " ";
// }
// std::cout << std::endl;

// std::string cleanName = solverName;
// std::replace(cleanName.begin(), cleanName.end(), ' ', '_');
// std::replace(cleanName.begin(), cleanName.end(), '(', '_');
// std::replace(cleanName.begin(), cleanName.end(), ')', '_');
//
// // Save best path to text file (always)
// std::string pathFile =
//     "solutions/paths/" + cleanName + "_" + instanceName + ".txt";
// std::ofstream pathOut(pathFile);
// if (pathOut.is_open()) {
//   pathOut << "Solver: " << solverName << std::endl;
//   pathOut << "Instance: " << instanceName << std::endl;
//   pathOut << "Total Cost: " << stats.minCost << std::endl;
//   pathOut << "Path Length: " << stats.bestSolution.getPathLength() <<
//   std::endl; pathOut << "Total Runtime (ms): " << stats.totalRuntimeMs <<
//   std::endl; pathOut << "Average Runtime (ms): " << stats.averageRuntimeMs <<
//   std::endl; pathOut << "Number of Runs: " << stats.numRuns << std::endl;
//   pathOut << std::endl;
//   pathOut << "Best Path:" << std::endl;
//   for (int node : stats.bestSolution.getSelectedNodes()) {
//     pathOut << node << " ";
//   }
//   pathOut << std::endl;
//   pathOut.close();
//   std::cout << "Path saved to: " << pathFile << std::endl;
// }
//
// if (enableVisualization) {
//   std::string outputFile =
//       "solutions/images/" + cleanName + "_" + instanceName + ".png";
//   TSPVisualizer::visualize(stats.bestSolution, tspaInstance,
//                            solverName + " - " + instanceName, outputFile);
// }

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <path_to_tspa_file> [--visualize]"
              << std::endl;
    return 1;
  }

  try {
    std::string tspaFilePath = argv[1];
    bool enableVisualization =
        (argc > 2 && std::string(argv[2]) == "--visualize");

    // Extract instance name from path
    std::string instanceName =
        std::filesystem::path(tspaFilePath).stem().string();

    std::filesystem::create_directories("solutions/paths");
    if (enableVisualization) {
      std::filesystem::create_directories("solutions/images");
    }

    TSPAInstance tspaInstanceA("data/TSPA.csv");
    TSPAInstance tspaInstanceB("data/TSPB.csv");

    std::cout << "Loaded " << tspaInstanceA.getTotalNodes() << " nodes from "
              << "data/TSPA.csv" << std::endl;
    std::cout << std::endl;
    std::cout << "Loaded " << tspaInstanceB.getTotalNodes() << " nodes from "
              << "data/TSPB.csv" << std::endl;
    std::cout << std::endl;

    // Helper function to print stats and optionally visualize
    auto printAndVisualize = [&](const std::string &solverName,
                                 const SolverStatistics &stats) {
      std::cout << "=== " << solverName << " ===" << std::endl;
      std::cout << "Total execution time (ms): " << stats.totalRuntimeMs
                << std::endl;
      std::cout << "Min run time (ms): " << stats.minRuntimeMs << std::endl;
      std::cout << "Max run time (ms): " << stats.maxRuntimeMs << std::endl;
      std::cout << "Average run time (ms): " << stats.averageRuntimeMs
                << std::endl;
      std::cout << "Number of runs: " << stats.numRuns << std::endl;
      std::cout << "Best cost: " << stats.minCost << std::endl;
      std::cout << "Worst cost: " << stats.maxCost << std::endl;
      std::cout << "Average cost: " << stats.averageCost << std::endl;
      std::cout << "Best path length: " << stats.bestSolution.getPathLength()
                << std::endl;
      std::cout << "Best path: " << std::endl;
      std::cout << solverName << " | " << stats.averageCost << " ("
                << stats.minCost << ", " << stats.maxCost << ")" << std::endl;
      std::cout << solverName << " | " << stats.averageRuntimeMs << " ("
                << stats.minRuntimeMs << ", " << stats.maxRuntimeMs << ")"
                << std::endl;
      for (int node : stats.bestSolution.getSelectedNodes()) {
        std::cout << node << " ";
      }
      std::cout << std::endl;

      std::string cleanName = solverName;
      std::replace(cleanName.begin(), cleanName.end(), ' ', '_');
      std::replace(cleanName.begin(), cleanName.end(), '(', '_');
      std::replace(cleanName.begin(), cleanName.end(), ')', '_');

      // Save best path to text file (always)
      std::string pathFile =
          "solutions/paths/" + cleanName + "_" + instanceName + ".txt";
      std::ofstream pathOut(pathFile);
      if (pathOut.is_open()) {
        pathOut << "Solver: " << solverName << std::endl;
        pathOut << "Instance: " << instanceName << std::endl;
        pathOut << "Total Cost: " << stats.minCost << std::endl;
        pathOut << "Path Length: " << stats.bestSolution.getPathLength()
                << std::endl;
        pathOut << "Total Runtime (ms): " << stats.totalRuntimeMs << std::endl;
        pathOut << "Average Runtime (ms): " << stats.averageRuntimeMs
                << std::endl;
        pathOut << "Number of Runs: " << stats.numRuns << std::endl;
        pathOut << std::endl;
        pathOut << "Best Path:" << std::endl;
        for (int node : stats.bestSolution.getSelectedNodes()) {
          pathOut << node << " ";
        }
        pathOut << std::endl;
        pathOut.close();
        std::cout << "Path saved to: " << pathFile << std::endl;
      }

      if (enableVisualization) {
        std::string outputFile =
            "solutions/images/" + cleanName + "_" + instanceName + ".png";
        TSPVisualizer::visualize(stats.bestSolution, tspaInstanceA,
                                 solverName + " - " + instanceName, outputFile);
      }
    };

    RandomSearch randomSearchSolver;
    NearestNeighbour nearestNeighbour;
    NearestNeighbourAny nearestNeighbourAny;
    NearestNeighbourCycle nearestNeighbourCycle;
    NearestNeighbour2Regret nearestNeighbour2Regret;
    kRegretCycle kRegretCycleSolver;
    kRegret kRegretSolver;

    std::vector<std::pair<SolverStatistics, SolverStatistics>> stats;
    std::vector<std::string> names;

    LocalSteepest LSStRandomEdgeSolver(LocalSteepest::StartingStrategy::Random,
                                       LocalSteepest::IntraMode::EdgeExchange);
    SolverStatistics lstatsEdgeA =
        SolverRunner::runAllStartingPoints(LSStRandomEdgeSolver, tspaInstanceA);

    SolverStatistics lstatsEdgeB =
        SolverRunner::runAllStartingPoints(LSStRandomEdgeSolver, tspaInstanceB);
    // printAndVisualize("Local Steepest Edge", lstatsEdgeB);
    stats.push_back({lstatsEdgeA, lstatsEdgeB});
    names.push_back("Local Steepest Edge");

    kRegretCycleWeighted kRegretCycleWeightedSolver;
    SolverStatistics krcwStatsA = SolverRunner::runAllStartingPoints(
        kRegretCycleWeightedSolver, tspaInstanceA);

    SolverStatistics krcwStatsB = SolverRunner::runAllStartingPoints(
        kRegretCycleWeightedSolver, tspaInstanceB);

    stats.push_back({krcwStatsA, krcwStatsB});
    names.push_back("kRegretCycleWeighted");

    printAndVisualizeBetter(stats, names, true, tspaInstanceA, tspaInstanceB);

    MSLS MSLSSolver(MSLS::StartingStrategy::Random,
                    MSLS::IntraMode::EdgeExchange);
    SolverStatistics lstatsMSLSA =
        SolverRunner::runAllStartingPoints(MSLSSolver, tspaInstanceA, 20);
    SolverStatistics lstatsMSLSB =
        SolverRunner::runAllStartingPoints(MSLSSolver, tspaInstanceB, 20);

    stats.push_back({lstatsMSLSA, lstatsMSLSB});
    names.push_back("MSLS");
    std::cout << "MSLS DONE" << std::endl;

    std::array<double, 5> probabilities = {0.1, 0.2, 0.3, 0.4, 0.5};
    std::array<bool, 2> useLS = {true, false};

    ILS ILSSolver(ILS::StartingStrategy::Random, ILS::IntraMode::EdgeExchange,
                  lstatsMSLSA.averageRuntimeMs, 15);
    SolverStatistics lstatsILSA =
        SolverRunner::runAllStartingPoints(ILSSolver, tspaInstanceA, 20);
    ILS ILSSolverB(ILS::StartingStrategy::Random, ILS::IntraMode::EdgeExchange,
                   lstatsMSLSB.averageRuntimeMs, 15);
    SolverStatistics lstatsILSB =
        SolverRunner::runAllStartingPoints(ILSSolverB, tspaInstanceB, 20);
    stats.push_back({lstatsILSA, lstatsILSB});
    names.push_back("ILS");
    std::cout << "ILS DONE" << std::endl;

    for (auto &p : probabilities) {
      for (auto &ls : useLS) {
        LNS LNSSolverA(LNS::StartingStrategy::Random,
                       LNS::IntraMode::EdgeExchange,
                       lstatsMSLSA.averageRuntimeMs, ls, p);
        SolverStatistics lstatsLNSA =
            SolverRunner::runAllStartingPoints(LNSSolverA, tspaInstanceA, 20);

        LNS LNSSolverB(LNS::StartingStrategy::Random,
                       LNS::IntraMode::EdgeExchange,
                       lstatsMSLSB.averageRuntimeMs, ls, p);
        SolverStatistics lstatsLNSB =
            SolverRunner::runAllStartingPoints(LNSSolverB, tspaInstanceB, 20);

        std::string ls_str = (ls ? "true" : "false");
        std::cout << "LNS " + ls_str + " " + std::to_string(p) << " DONE"
                  << std::endl;
        stats.push_back({lstatsLNSA, lstatsLNSB});
        names.push_back("LNS " + ls_str + " " + std::to_string(p));
      }
    }

    printAndVisualizeBetter(stats, names, false, tspaInstanceA, tspaInstanceB);
    // std::cout << std::endl;
    // printAndVisualize("LNS 15", lstatsLNS);

    // MSLS MSLSSolver(MSLS::StartingStrategy::Random,
    // MSLS::IntraMode::EdgeExchange); SolverStatistics lstatsMSLS =
    // SolverRunner::runAllStartingPoints(MSLSSolver, tspaInstance, 20);
    // printAndVisualize("MSLS", lstatsMSLS);
    //
    // ILS ILSSolver(ILS::StartingStrategy::Random,
    // ILS::IntraMode::EdgeExchange, lstatsMSLS.averageRuntimeMs, 15);
    // SolverStatistics lstatsILS =
    // SolverRunner::runAllStartingPoints(ILSSolver, tspaInstance, 20);
    // std::cout << std::endl;
    // printAndVisualize("ILS 15", lstatsILS);

    // LocalSteepest
    // LSStHeuristicNodeSolver(LocalSteepest::StartingStrategy::Heuristic,
    // LocalSteepest::IntraMode::NodeExchange); SolverStatistics lstatsHeuNode =
    // SolverRunner::runAllStartingPoints(LSStHeuristicNodeSolver,
    // tspaInstance); printAndVisualize("Local Steepest Heuristic Node",
    // lstatsHeuNode);

    // LocalSteepest
    // LSStHeuristicEdgeSolver(LocalSteepest::StartingStrategy::Heuristic,
    // LocalSteepest::IntraMode::EdgeExchange); SolverStatistics lstatsHeuEdge =
    // SolverRunner::runAllStartingPoints(LSStHeuristicEdgeSolver,
    // tspaInstance); printAndVisualize("Local Steepest Heuristic Edge",
    // lstatsHeuEdge);

    // PrevDeltas prevDeltasSolver(PrevDeltas::StartingStrategy::Random);
    // SolverStatistics pdStats =
    // SolverRunner::runAllStartingPoints(prevDeltasSolver, tspaInstance);
    // printAndVisualize("Prev Deltas", pdStats);

  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
