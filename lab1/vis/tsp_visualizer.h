#ifndef TSP_VISUALIZER_H
#define TSP_VISUALIZER_H

#include "tsp_parser.h"
#include <matplot/matplot.h>
#include <string>
#include <vector>
#include <algorithm>
#include <unordered_set>

class TSPVisualizer {
private:
    std::vector<TSPNode> solutionNodes;
    std::vector<TSPNode> allNodes;
    std::string title;
    bool showAllNodes;
    
public:
    TSPVisualizer(const std::vector<TSPNode>& nodes, const std::string& title = "TSP Solution")
        : solutionNodes(nodes), allNodes(nodes), title(title), showAllNodes(false) {}
        
    TSPVisualizer(const std::vector<TSPNode>& solutionNodes, 
                 const std::vector<TSPNode>& allNodes,
                 const std::string& title = "TSP Solution")
        : solutionNodes(solutionNodes), allNodes(allNodes), title(title), showAllNodes(true) {}
    
    void visualize(const std::string& outputFile = "") {
        using namespace matplot;
        
        auto f = figure(true);
        hold(on);
        
        // First, if showing all nodes, plot the unused nodes as small gray dots
        if (showAllNodes) {
            std::vector<double> unused_x, unused_y;
            std::vector<double> unused_costs;
            
            std::unordered_set<int> solutionIndices;
            for (const auto& node : solutionNodes) {
                solutionIndices.insert(node.id);
            }
            
            // Collect unused nodes
            for (const auto& node : allNodes) {
                if (solutionIndices.find(node.id) == solutionIndices.end()) {
                    unused_x.push_back(node.x);
                    unused_y.push_back(node.y);
                    unused_costs.push_back(node.cost);
                }
            }
            
            if (!unused_x.empty()) {
                auto unused_sc = scatter(unused_x, unused_y);
                unused_sc->marker_size(4);
                unused_sc->marker_face(true);
                unused_sc->marker_color({0.7, 0.7, 0.7});
            }
        }
        
        std::vector<double> x_coords, y_coords;
        for (const auto& node : solutionNodes) {
            x_coords.push_back(node.x);
            y_coords.push_back(node.y);
        }
        
        x_coords.push_back(solutionNodes[0].x);
        y_coords.push_back(solutionNodes[0].y);
        
        auto path_plot = plot(x_coords, y_coords, "-");
        path_plot->line_width(2.0);
        path_plot->color({0.0, 0.0, 0.0});
        
        auto [min_it, max_it] = std::minmax_element(
            solutionNodes.begin(), solutionNodes.end(),
            [](const TSPNode& a, const TSPNode& b) { return a.cost < b.cost; }
        );
        double min_cost = min_it->cost;
        double max_cost = max_it->cost;
        
        std::vector<double> node_x(x_coords.begin(), x_coords.end() - 1);
        std::vector<double> node_y(y_coords.begin(), y_coords.end() - 1);
        std::vector<double> costs;
        std::vector<double> sizes;
        
        for (const auto& node : solutionNodes) {
            costs.push_back(node.cost);
            
            // Normalize size between 15 and 50 based on cost
            // double norm_cost = (node.cost - min_cost) / (max_cost - min_cost);
            // sizes.push_back(15 + norm_cost * 35); // Slightly larger minimum size
            sizes.push_back(15);
        }
        
        colormap(matplot::palette::jet());
        auto sc = scatter(node_x, node_y, sizes, costs);
        sc->marker_face(true);
        
        for (size_t i = 0; i < solutionNodes.size(); i++) {
            auto t = text(solutionNodes[i].x+1, solutionNodes[i].y, std::to_string(i));
            t->font_size(15);
            t->color({0.0, 0.0, 0.0});
        }

        xlabel("X Coordinate");
        ylabel("Y Coordinate");
        matplot::title(this->title + " (Total Distance: " + 
              std::to_string(TSPParser::calculateTotalDistance(solutionNodes)) + ")");

        auto cb = colorbar();
        cb.label("Node Cost");

        if (!outputFile.empty()) {
            save(outputFile);
        }
        
        show();
    }
};

#endif