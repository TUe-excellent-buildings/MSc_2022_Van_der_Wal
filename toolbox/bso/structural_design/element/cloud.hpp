#ifndef SD_CLOUD_HPP
#define SD_CLOUD_HPP

#include <bso/utilities/geometry/vertex.hpp>
#include <bso/structural_design/component/load.hpp>
#include <bso/structural_design/element/node.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <vector>
#include <map>

namespace bso { namespace structural_design { namespace element {
	
	class cloud : public bso::utilities::geometry::vertex
	{
	private:
		unsigned long mID; // the ID of this cloud
		
	public:
		template<class T>
		cloud(const unsigned long& ID, const Eigen::MatrixBase<T>& rhs);
		virtual ~cloud();
		
		std::vector<node*> slaveNodes; // pointers to the slave nodes of this cloud
		node* masterNode;
		std::vector<long> slaveDOFs;
		std::vector<long> masterDOFs;
		
		const unsigned long& ID() const {return mID;}
	};
	
} // namespace element
} // namespace structural_design
} // namespace bso

#include <bso/structural_design/element/cloud.cpp>

#endif // SD_CLOUD_HPP