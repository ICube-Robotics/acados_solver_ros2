#include <string>
#include <unordered_map>
#include <vector>

namespace acados
{
using IndexMap = std::unordered_map<std::string, std::vector<unsigned int>>;
using ValueMap = std::unordered_map<std::string, std::vector<double>>;
} // namespace acados
