#include "px4_offboard_controllers/wrench/normalization.hpp"
#include <cassert>
using namespace px4_offboard;
int main(){ WrenchNormalizationResult r{true,"",{.1,.2,.3},{0,0,-.7}}; auto c=toPx4WrenchCommand(r,10); assert(c&&c->valid()); WrenchNormalizationResult bad{false,"residual",{}, {}}; assert(!toPx4WrenchCommand(bad,10)); }
