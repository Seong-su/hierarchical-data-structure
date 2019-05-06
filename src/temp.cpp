#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <queue>
#include <string>
#include <unordered_set>


namespace fs = std::filesystem;
using namespace std;

// class Member {
// private:
//   const string name_;
//   const BoundingBox bounding_box_;

// public:
//   Member(string name, aiMesh *mesh) : name_(name), bounding_box_(mesh) {}
//   ~Member() {}

//   const string &Name() const { return name_; }
//   const BoundingBox &Box() const { return bounding_box_; }
// };

// class Edge {
// private:
//   const Member *m1_, *m2_;
//   const BoundingBox *n1_, *n2_;
//   ai_real attraction_, cost_;

// public:
//   Edge(Member *m1, Member *m2) {
//     m1_ = m1;
//     m2_ = m2;
//     n1_ = &m1->Box();
//     n2_ = &m2->Box();

//     ai_real distance = (n1_->Mid() - n2_->Mid()).Length();
//     BoundingBox n12(n1_,
//                     n2_); // TODO: n12 must be reserved for muli-layer level
//     attraction_ = n1_->Radius() * n2_->Radius() / (distance * distance);
//     cost_ = (n12.Radius() * n12.Radius() * n12.Radius()) / attraction_;
//   }
//   ~Edge() {}

//   friend bool operator<(const Edge &e1, const Edge &e2) {
//     return e1.Cost() < e2.Cost();
//   }

//   const Member &M1() const { return *m1_; }
//   const Member &M2() const { return *m2_; }
//   const ai_real Attraction() const { return attraction_; }
//   const ai_real Cost() const { return cost_; }
// };

// int main(int, char **) {
//   vector<Member> nodes; // TODO: Should know the num of models in directory
//                         // (to reserve memory)

//   string model_directory("parts-model");
//   {
//     int num_model_files_in_directory = 0, num_files_in_directory = 0;

//     for (auto &p : fs::directory_iterator(model_directory)) {
//       ++num_files_in_directory;

//       bool is_stl = true; // TODO: read only files in stl extension.

//       if (is_stl) {
//         Assimp::Importer importer;

//         const aiScene *scene = importer.ReadFile(
//             p.path().string(), aiProcess_CalcTangentSpace |
//                                    aiProcess_JoinIdenticalVertices |
//                                    aiProcess_SortByPType);
//         if (!scene) {
//           cerr << "[Error] Cannot read " << p.path() << " file!\n";
//         }

//         size_t num_meshes = scene->mNumMeshes;
//         for (size_t mesh_idx = 0; mesh_idx < num_meshes; ++mesh_idx) {
//           nodes.emplace_back(
//               p.path().stem().string(),
//               scene->mMeshes[mesh_idx]); // TODO: move and forward for high
//                                          // performance.
//         }

//         ++num_model_files_in_directory;
//       }
//     }

//     cout << "Succesfully load " << num_model_files_in_directory
//          << " model file(s) out of " << num_files_in_directory
//          << " file(s) in the directory(" << model_directory << ")\n";
//   }

//   vector<Edge> edges; // adjecency graph
//   {
//     for (auto n1 = nodes.begin(); n1 != nodes.end(); ++n1) {
//       for (auto n2 = (n1 + 1); n2 != nodes.end(); ++n2) {
//         edges.emplace_back(&*n1, &*n2);
//       }
//     }

//     sort(edges.begin(), edges.end());

//     for (auto &e : edges) {
//       cout << "The attraction between " << e.M1().Name() << " and "
//            << e.M2().Name() << " is " << e.Attraction() << " and the cost is
//            "
//            << e.Cost() << endl;
//     }
//   }

//   priority_queue<Edge *> minimum_spanning_tree;
//   {
//     unordered_set<string> visited;
//     minimum_spanning_tree.push(&edges.at(0)); // TODO: edges should not empty

//     while (!minimum_spanning_tree.empty()) {
//       const Member &m1 = minimum_spanning_tree.top()->M1();
//       const Member &m2 = minimum_spanning_tree.top()->M2();
//       const ai_real cost = minimum_spanning_tree.top()->Cost();
//       minimum_spanning_tree.pop();

//       if (!visited.insert(m1.Name()).second) {
//         continue;
//       }

//       for (auto &e : edges) {
//         if (&e.M1() == &m1) {
//           if (visited.find(e.M2().Name()) == visited.end()) {
//             minimum_spanning_tree.push(&e);
//             cout << "The cost between " << e.M1().Name() << " and "
//                  << e.M2().Name() << " is " << e.Cost() << endl;
//           }
//         }
//       }
//     }

//     while (!minimum_spanning_tree.empty()) {
//       auto *e = minimum_spanning_tree.top();
//       cout << "The cost between " << e->M1().Name() << " and " <<
//       e->M2().Name()
//            << " is " << e->Cost() << endl;

//       minimum_spanning_tree.pop();
//     }
//   }

//   cout << "Hello, world!\n";

//   return 0;
// }

// #include <boost/graph/adj_list_serialize.hpp>
// #include <boost/graph/graph_utility.hpp>
// #include <boost/graph/graphviz.hpp>
// #include <boost/property_map/function_property_map.hpp>
// #include <boost/property_map/transform_value_property_map.hpp>
// #include <boost/variant.hpp>
// #include <fstream>
// using namespace boost;

// namespace Nodes {
// struct Building {
//   std::string id;
//   double weight;
// };
// struct Contract {
//   std::string id;
// };

// static inline std::ostream& operator<<(std::ostream& os, Building const& b) {
//   return os << "Building " << b.id;
// }
// static inline std::ostream& operator<<(std::ostream& os, Contract const& b) {
//   return os << "Contract " << b.id;
// }

// std::string id_of(Building const& b) { return b.id; }
// std::string id_of(Contract const& c) { return c.id; }
// std::string shape_of(Building const& b) { return "circle"; }
// std::string shape_of(Contract const& c) { return "diamond"; }
// }  // namespace Nodes

// using Nodes::Building;
// using Nodes::Contract;
// using Vertex = boost::variant<Building, Contract>;

// std::string id_of(Vertex const& v) {
//   return boost::apply_visitor([](auto const& node) { return id_of(node); },
//   v);
// }
// std::string shape_of(Vertex const& v) {
//   return boost::apply_visitor([](auto const& node) { return shape_of(node);
//   },
//                               v);
// }

// typedef adjacency_list<listS, listS, undirectedS, Vertex> Graph;

// int main() {
//   Graph g;
//   auto office1 = add_vertex(Building{"office1"}, g);
//   auto office2 = add_vertex(Building{"office2"}, g);
//   auto warehouse1 = add_vertex(Building{"warehouse1"}, g);
//   auto contract1 = add_vertex(Contract{"contract1"}, g);
//   auto contract2 = add_vertex(Contract{"contract2"}, g);

//   add_edge(office1, office2, g);
//   add_edge(warehouse1, contract1, g);
//   add_edge(contract2, contract1, g);

//   {
//     std::ofstream dot_file("figs/graph.dot");
//     dynamic_properties dp;

//     dp.property("node_id", boost::make_transform_value_property_map(
//                                &::id_of, boost::get(boost::vertex_bundle,
//                                g)));
//     dp.property("shape", boost::make_transform_value_property_map(
//                              &::shape_of, boost::get(boost::vertex_bundle,
//                              g)));
//     dp.property("label", boost::make_transform_value_property_map(
//                              [](Vertex const& v) {
//                                return boost::lexical_cast<std::string>(v);
//                              },
//                              boost::get(boost::vertex_bundle, g)));

//     write_graphviz_dp(dot_file, g, dp);
//   }

//   print_graph(g, make_transform_value_property_map(
//                      &::id_of, get(boost::vertex_bundle, g)));

//   return EXIT_SUCCESS;
// }

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/property_map/function_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/variant.hpp>

using namespace boost;

namespace Nodes {
struct Block {
  std::string name;
  int id;
  double weight;
};

static inline std::ostream& operator<<(std::ostream& os, Block const& b) {
  return os << "Block " << b.name;
}

int id_of(Block const& b) { return b.id; }
std::string name_of(Block const& b) { return b.name; }
std::string shape_of(Block const& b) { return "circle"; }
}  // namespace Nodes

using Nodes::Block;

typedef adjacency_list<listS, listS, undirectedS, Block> Graph;

template <class Graph>
struct exercise_vertex {
  exercise_vertex(Graph& g_) : g(g_) {}
  Graph& g;

  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;

  void operator()(Vertex v) const {
    std::cout << "adjacent vertices: ";
    typename property_map<Graph, std::string Nodes::Block::*>::type name =
        get(&Block::name, g);
    typename graph_traits<Graph>::adjacency_iterator ai;
    typename graph_traits<Graph>::adjacency_iterator ai_end;
    for (boost::tie(ai, ai_end) = adjacent_vertices(v, g); ai != ai_end; ++ai)
      std::cout << name[*ai] << " ";
    std::cout << std::endl;
  }
};

int main() {

  //-------------------------------------------------------------------------//
  Graph g;
  auto office1 = add_vertex(Block{"office1"}, g);
  auto office2 = add_vertex(Block{"office2"}, g);
  auto warehouse1 = add_vertex(Block{"warehouse1"}, g);

  // get the property map for vertex indices
  auto names = get(&Nodes::Block::name, g);

  std::cout << "vertices(g) = ";
  typedef graph_traits<Graph>::vertex_iterator vertex_iter;
  std::pair<vertex_iter, vertex_iter> vp = vertices(g);

  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;

  // std::for_each(vp.first, vp.second, exercise_vertex<Graph>(g));
  std::for_each(vp.first, vp.second,
                [&](const Vertex& v) { std::cout << names[v] << " "; });
  // for (vp = vertices(g); vp.first != vp.second; ++vp.first)
  //   std::cout << names[*vp.first] << " ";
  std::cout << "|" << names[*vp.first] << "|";
  std::cout << std::endl;

  auto edge1 = add_edge(warehouse1, office1, g);
  add_edge(warehouse1, office2, g);

  {
    std::ofstream dot_file("bin/graph.dot");
    dynamic_properties dp;

    dp.property("node_id", make_transform_value_property_map(
                               &Nodes::id_of, get(vertex_bundle, g)));
    dp.property("shape", make_transform_value_property_map(
                             &Nodes::shape_of, get(vertex_bundle, g)));
    dp.property("label", make_transform_value_property_map(
                             [](Block const& v) {
                               return lexical_cast<std::string>(v.id) + "-" +
                                      v.name;
                             },
                             get(vertex_bundle, g)));

    write_graphviz_dp(dot_file, g, dp);
  }

  print_graph(g, get(&Nodes::Block::name, g));

  return EXIT_SUCCESS;
}