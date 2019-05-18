#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/property_map/function_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/variant.hpp>

#include "BCTNode.hpp"
#include "BoundingBox.hpp"
#include "Connection.hpp"

using namespace boost;

typedef adjacency_list<listS, listS, undirectedS, BoundingBox, Connection>
    Graph;
typedef typename graph_traits<Graph>::vertex_descriptor VertexDesc;
typedef typename graph_traits<Graph>::edge_descriptor EdgeDesc;
typedef typename graph_traits<Graph>::vertex_iterator VertexIter;
typedef typename graph_traits<Graph>::edge_iterator EdgeIter;

template <class Graph>
struct exercise_vertex {
  exercise_vertex(Graph& g_) : g(g_) {}
  Graph& g;

  void operator()(VertexDesc v) const {
    std::cout << "adjacent vertices: ";
    typename property_map<Graph, std::string BoundingBox::*>::type name =
        get(&BoundingBox::name_, g);
    typename graph_traits<Graph>::adjacency_iterator ai;
    typename graph_traits<Graph>::adjacency_iterator ai_end;
    for (boost::tie(ai, ai_end) = adjacent_vertices(v, g); ai != ai_end; ++ai)
      std::cout << name[*ai] << " ";
    std::cout << std::endl;
  }
};

int main() {
  using std::cerr;
  using std::cout;
  using std::endl;
  using std::string;
  using std::vector;

  namespace fs = std::filesystem;

  Graph graph;
  string model_directory("res/parts-model");
  {
    int num_model_files_in_directory = 0, num_files_in_directory = 0;

    for (auto& p : fs::directory_iterator(model_directory)) {
      ++num_files_in_directory;

      if (p.path().extension().string() == ".stl") {
        Assimp::Importer importer;

        const aiScene* scene = importer.ReadFile(
            p.path().string(), aiProcess_CalcTangentSpace |
                                   aiProcess_JoinIdenticalVertices |
                                   aiProcess_SortByPType);
        if (!scene) {
          cerr << "[Error] Cannot read " << p.path() << " file!\n";
        } else {
          size_t num_meshes = scene->mNumMeshes;
          std::for_each(
              scene->mMeshes, scene->mMeshes + num_meshes,
              [num_model_files_in_directory, &p, &graph](aiMesh* mesh) {
                add_vertex(BoundingBox(num_model_files_in_directory,
                                       p.path().stem().string(), mesh),
                           graph);
              });
          ++num_model_files_in_directory;
        }
      }
    }

    cout << "Succesfully load " << num_model_files_in_directory
         << " model file(s) out of " << num_files_in_directory
         << " file(s) in the directory(" << model_directory << ")\n";

    auto names = get(&BoundingBox::name_, graph);
    cout << "vertices(g) = ";
    std::pair<VertexIter, VertexIter> vp = vertices(graph);
    std::for_each(vp.first, vp.second,
                  [&names](const VertexDesc& v) { cout << names[v] << " "; });
    cout << endl;

    auto maxs = get(&BoundingBox::max_, graph);
    auto mins = get(&BoundingBox::min_, graph);

    for (VertexIter vert1 = vp.first; vert1 != vp.second; ++vert1) {
      for (VertexIter vert2 = std::next(vert1, 1); vert2 != vp.second;
           ++vert2) {
        bool intersected = mins[*vert1].x <= maxs[*vert2].x &&
                           maxs[*vert1].x >= mins[*vert2].x &&
                           mins[*vert1].y <= maxs[*vert2].y &&
                           maxs[*vert1].y >= mins[*vert2].y &&
                           mins[*vert1].z <= maxs[*vert2].z &&
                           maxs[*vert1].z >= mins[*vert2].z;

        if (intersected) {
          cout << names[*vert1] << " " << names[*vert2] << "\n";
          add_edge(*vert1, *vert2, Connection(&graph[*vert1], &graph[*vert2]),
                   graph);
        }
      }
    }

    auto edge_names = get(&BoundingBox::name_, graph);
    cout << "edges(g) = ";
    graph_traits<Graph>::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei) {
      cout << "(" << edge_names[source(*ei, graph)] << ","
           << edge_names[target(*ei, graph)] << ") ";
    }
    cout << endl;
  }

  {
    std::ofstream dot_file("bin/graph.dot");
    dynamic_properties dp;

    dp.property("node_id", make_transform_value_property_map(
                               &BoundingBox::id, get(vertex_bundle, graph)));
    dp.property("shape", make_transform_value_property_map(
                             [&](const BoundingBox& box) { return "circle"; },
                             get(vertex_bundle, graph)));
    dp.property("label", make_transform_value_property_map(
                             &BoundingBox::name, get(vertex_bundle, graph)));

    write_graphviz_dp(dot_file, graph, dp);
  }

  vector<EdgeDesc> spanning_tree;
  spanning_tree.reserve(num_vertices(graph));
  {
    auto costs = get(&Connection::cost_, graph);
    auto edge_names = get(&BoundingBox::id_, graph);
    kruskal_minimum_spanning_tree(
        graph, std::back_inserter(spanning_tree),
        weight_map(costs).vertex_index_map(get(&BoundingBox::id_, graph)));

    cout << "Print the edges in the MST:" << endl;
    for (vector<EdgeDesc>::iterator ei = spanning_tree.begin();
         ei != spanning_tree.end(); ++ei) {
      cout << edge_names[source(*ei, graph)] << " <--> "
           << edge_names[target(*ei, graph)] << " with weight of " << costs[*ei]
           << endl;
    }

    std::ofstream fout("bin/kruskal-eg.dot");
    fout << "graph g {\n"
         << " ranksep=0.5\n"
         << " size=\"50,10\"\n"
         << " ratio=\"fill\"\n"
         << " edge[dir=none]\n"
         << " node[shape=\"box3d\", width=1.5, height=0.5, "
            "fontname=\"Arial\"]\n";
    VertexIter viter, viter_end;
    for (boost::tie(viter, viter_end) = vertices(graph); viter != viter_end;
         ++viter) {
      fout << graph[*viter].id_ << "[label=\"" << graph[*viter].name_
           << "\"];\n";
    }
    EdgeIter eiter, eiter_end;
    for (boost::tie(eiter, eiter_end) = edges(graph); eiter != eiter_end;
         ++eiter) {
      if (std::find(spanning_tree.begin(), spanning_tree.end(), *eiter) !=
          spanning_tree.end()) {
        fout << edge_names[source(*eiter, graph)] << " -- "
             << edge_names[target(*eiter, graph)];
        fout << "[color=\"red\", label=\""
             << get(&Connection::cost_, graph, *eiter) << "\"];\n";

      } else {
        fout << edge_names[source(*eiter, graph)] << " -- "
             << edge_names[target(*eiter, graph)];
        fout << "[color=\"gray\", label=\""
             << get(&Connection::cost_, graph, *eiter) << "\"];\n";
      }
    }
    fout << "}\n";
  }

  BCTForest forest;
  {
    auto edge_names = get(&BoundingBox::name_, graph);
    int idx = 0;
    for (auto ei = spanning_tree.begin(); ei != spanning_tree.end(); ++ei) {
      auto node =
          std::make_unique<BCTNode>(edge_names[source(*ei, graph)],
                                    edge_names[target(*ei, graph)], ++idx);
      forest.insert(std::move(node));
    }

    forest.traversal();
  }

  return EXIT_SUCCESS;
}
