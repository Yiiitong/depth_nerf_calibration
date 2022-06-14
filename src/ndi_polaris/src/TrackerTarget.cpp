
#include <ndi_polaris/convert_string.hpp>
#include <ndi_polaris/TrackerTarget.hpp>


TrackerTarget::TrackerTarget() :
    m_id(0),
    m_trans(),
    m_status(TrackerTarget::NO),
    m_error(0.0f)
{
}


TrackerTarget::TrackerTarget( const TrackerTarget& target ) :
    m_id(target.m_id),
    m_trans(target.m_trans),
    m_status(target.m_status),
    m_error(target.m_error)
{}


TrackerTarget::TrackerTarget( size_t id, const Eigen::Affine3f &trans, int status, float error )
{
    set( id, trans, status, error );
}


TrackerTarget::TrackerTarget( size_t id, const Eigen::Matrix4f &trans, int status, float error )
{
    set( id, trans, status, error );
}


TrackerTarget::TrackerTarget( size_t id, const Eigen::Quaternionf &rot, const Eigen::Vector3f &transl, int status, float error )
{
    set( id, rot, transl, status, error );
}


TrackerTarget::~TrackerTarget()
{
}


size_t TrackerTarget::id() const
{
    return m_id;
}


const Eigen::Affine3f &TrackerTarget::trans() const
{
    return m_trans;
}

std::vector<float> TrackerTarget::mat()
{
    std::vector<float> fl;
    for (int i=0 ; i<4 ;i++)
    {
        for (int j=0 ; j<4 ; j++)
        {
            fl.push_back(m_trans(i,j));
        }
    }
    return fl;
}



int TrackerTarget::status() const
{
    return m_status;
}


float TrackerTarget::error() const
{
    return fabs(m_error);
}


//bool TrackerTarget::isTracked() const
//{
//    return isTracked( 0.5f );
//}


bool TrackerTarget::isTracked( float maxError ) const
{
    if( (m_status == TrackerTarget::OK || m_status == TrackerTarget::Partial ) && fabs(m_error) <= maxError )
        return true;
    else
        return false;
}


void TrackerTarget::set( size_t id )
{
    m_id = id;
}


void TrackerTarget::set( size_t id, const Eigen::Affine3f &trans, int status, float error )
{
    m_id = id;
    m_trans = trans;
    m_status = status;
    m_error = error;
}


void TrackerTarget::set( size_t id, const Eigen::Matrix4f &trans, int status, float error )
{
    m_id = id;
    m_trans = trans;
    m_status = status;
    m_error = error;
}


void TrackerTarget::set( size_t id, const Eigen::Quaternionf &rot, const Eigen::Vector3f &transl, int status, float error )
{
    m_id = id;

    // set rotation & translation
    m_trans.setIdentity();
    m_trans.translate( transl );
    m_trans.rotate( rot );

    m_status = status;
    m_error = error;
}

//} // end namespace eos
