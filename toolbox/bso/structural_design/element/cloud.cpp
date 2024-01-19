#ifndef SD_CLOUD_CPP
#define SD_CLOUD_CPP

namespace bso { namespace structural_design { namespace element {

	template<class T>
	cloud::cloud(const unsigned long& ID, const Eigen::MatrixBase<T>& rhs) :
		bso::utilities::geometry::vertex(rhs)
	{ //
		mID = ID;
	} // ctor

	cloud::~cloud()
	{ //
		
	} // dtor

} // namespace element
} // namespace structural_design
} // namespace bso

#endif // SD_CLOUD_CPP