#ifndef TSPVISUALIZER_H
#define TSPVISUALIZER_H

#include "TSPAInstance.h"
#include "Solution.h"
#include <matplot/matplot.h>
#include <string>
#include <vector>
#include <algorithm>
#include <unordered_set>

class TSPVisualizer {
public:
    static void visualize(const Solution& solution, 
                         const TSPAInstance& instance,
                         const std::string& title = "TSP Solution",
                         const std::string& outputFile = "") {
        using namespace matplot;
        
        auto f = figure(true);
        f->size(1200, 900);  // Larger figure size
        hold(on);
        
        const auto& selectedNodes = solution.getSelectedNodes();
        // const std::vector<int> selectedNodes = {
        //     95,183,140,4,149,28,20,60,148,47,94,179,166,194,176,113,103,127,89,163,153,81,77,141,91,61,36,177,21,82,111,8,104,56,144,160,33,49,11,139,138,182,25,5,142,78,175,80,190,73,54,31,193,117,198,1,16,27,38,63,135,122,90,51,121,116,98,118,74,134,6,188,169,132,70,3,15,145,13,195,168,29,0,109,35,143,159,106,124,62,18,34,55,83,128,86,185,22,99,130
        // };
        int totalNodes = instance.getTotalNodes();
        
        // Collect unused nodes
        std::vector<double> unused_x, unused_y, unused_costs;
        std::unordered_set<int> selectedSet(selectedNodes.begin(), selectedNodes.end());
        
        for (int i = 0; i < totalNodes; i++) {
            if (selectedSet.find(i) == selectedSet.end()) {
                unused_x.push_back(instance.getNodeX(i));
                unused_y.push_back(instance.getNodeY(i));
                unused_costs.push_back(instance.getNodeCost(i));
            }
        }
        
        // Build solution coordinates and path
        std::vector<double> x_coords, y_coords, costs;
        for (int node : selectedNodes) {
            x_coords.push_back(instance.getNodeX(node));
            y_coords.push_back(instance.getNodeY(node));
            costs.push_back(instance.getNodeCost(node));
        }
        
        // Close the cycle
        x_coords.push_back(instance.getNodeX(selectedNodes[0]));
        y_coords.push_back(instance.getNodeY(selectedNodes[0]));
        
        // Plot path
        auto path_plot = plot(x_coords, y_coords, "-");
        path_plot->line_width(2.0);
        path_plot->color({0.0, 0.0, 0.0});
        
        // Calculate global color range
        double global_min = *std::min_element(costs.begin(), costs.end());
        double global_max = *std::max_element(costs.begin(), costs.end());
        
        if (!unused_costs.empty()) {
            auto [um_min, um_max] = std::minmax_element(unused_costs.begin(), unused_costs.end());
            global_min = std::min(global_min, *um_min);
            global_max = std::max(global_max, *um_max);
        }
        
        if (global_min == global_max) {
            global_min -= 0.5;
            global_max += 0.5;
        }
        
        colormap(matplot::palette::jet());
        matplot::caxis({global_min, global_max});
        
        // Plot solution nodes (exclude the closing duplicate)
        std::vector<double> node_x(x_coords.begin(), x_coords.end() - 1);
        std::vector<double> node_y(y_coords.begin(), y_coords.end() - 1);
        
        auto sc = scatter(node_x, node_y, 10, costs);
        sc->marker_face(true);
        
        // Plot unused nodes with different appearance
        if (!unused_x.empty()) {
            auto unused_sc = scatter(unused_x, unused_y, 8, unused_costs);  // Slightly smaller
            unused_sc->marker_size(8);
            unused_sc->marker_face(true);
            unused_sc->marker_face_alpha(0.3);  // More transparent
        }
        
        // Add node labels only for selected nodes, with better positioning
        for (size_t i = 0; i < selectedNodes.size(); i++) {
            double offset_x = 3.0;  // Adjust based on your coordinate scale
            double offset_y = 3.0;
            auto t = text(instance.getNodeX(selectedNodes[i]) + offset_x, 
                         instance.getNodeY(selectedNodes[i]) + offset_y, 
                         std::to_string(selectedNodes[i]));
            t->font_size(10);
            t->color({0.0, 0.0, 0.0});
        }
        
        xlabel("X Coordinate");
        ylabel("Y Coordinate");
        
        std::string fullTitle = title + " (Cost: " + std::to_string((int)solution.getTotalCost()) + 
                               ", Length: " + std::to_string((int)solution.getPathLength()) + ")";
        matplot::title(fullTitle);
        
        // Add padding to axes
        auto ax = gca();
        double x_min = *std::min_element(x_coords.begin(), x_coords.end());
        double x_max = *std::max_element(x_coords.begin(), x_coords.end());
        double y_min = *std::min_element(y_coords.begin(), y_coords.end());
        double y_max = *std::max_element(y_coords.begin(), y_coords.end());
        
        if (!unused_x.empty()) {
            x_min = std::min(x_min, *std::min_element(unused_x.begin(), unused_x.end()));
            x_max = std::max(x_max, *std::max_element(unused_x.begin(), unused_x.end()));
            y_min = std::min(y_min, *std::min_element(unused_y.begin(), unused_y.end()));
            y_max = std::max(y_max, *std::max_element(unused_y.begin(), unused_y.end()));
        }
        
        double x_margin = (x_max - x_min) * 0.1;  // 10% margin
        double y_margin = (y_max - y_min) * 0.1;
        xlim({x_min - x_margin, x_max + x_margin});
        ylim({y_min - y_margin, y_max + y_margin});
        
        auto cb = colorbar();
        cb.label("Node Cost");
        
        if (!outputFile.empty()) {
            save(outputFile);
            std::cout << "Plot saved to: " << outputFile << std::endl;
        } else {
            show();
        }
    }
};

#endif // TSPVISUALIZER_H
