#include <rokubimini_cosmo/RokubiminiCosmo.hpp>

int main(int argc, char** argv)
{
  any_node::Nodewrap<rokubimini_cosmo::RokubiminiCosmo> node(argc, argv, "rokubimini_cosmo", 2, true);
  node.execute();
  return 0;
}
