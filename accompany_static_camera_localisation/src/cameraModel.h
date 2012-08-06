/***************************************************************************
 *   cameraModel.h     - CameraModel CLASS DEFINITION
 *
 *   Ninghang Hu - University of Amsterdam
 *
 *   ACCOMPANY PROJECT 2012
 *
 ***************************************************************************/

#ifndef _CAMERA_MODEL_OBJECT_H_
#define _CAMERA_MODEL_OBJECT_H_

#include "xmlUtil.h"

#include "opencv2/opencv.hpp"
#include "string"

using namespace std;

#define XML_TAG_CAMERA			BAD_CAST"Camera"
#define XML_TAG_NAME			BAD_CAST"name"

#define XML_TAG_GEOMETRY		BAD_CAST"Geometry"
#define XML_TAG_WIDTH			BAD_CAST"width"
#define XML_TAG_HEIGHT			BAD_CAST"height"
#define XML_TAG_NCX				BAD_CAST"ncx"
#define XML_TAG_NFX				BAD_CAST"nfx"
#define XML_TAG_DX				BAD_CAST"dx"
#define XML_TAG_DY				BAD_CAST"dy"
#define XML_TAG_DPX				BAD_CAST"dpx"
#define XML_TAG_DPY				BAD_CAST"dpy"

#define XML_TAG_INTRINSIC		BAD_CAST"Intrinsic"
#define XML_TAG_FOCAL			BAD_CAST"focal" 
#define XML_TAG_KAPPA1			BAD_CAST"kappa1" 
#define XML_TAG_CX				BAD_CAST"cx" 
#define XML_TAG_CY				BAD_CAST"cy" 
#define XML_TAG_SX				BAD_CAST"sx" 

#define XML_TAG_EXTRINSIC		BAD_CAST"Extrinsic"
#define XML_TAG_TX				BAD_CAST"tx" 
#define XML_TAG_TY				BAD_CAST"ty" 
#define XML_TAG_TZ				BAD_CAST"tz" 
#define XML_TAG_RX				BAD_CAST"rx" 
#define XML_TAG_RY				BAD_CAST"ry" 
#define XML_TAG_RZ				BAD_CAST"rz" 

namespace Hu
{

  //!  A root class handling camera model
  class CameraModel
  {
    public:

      /* PRIVATE */ // TODO
      bool isInit;
      cv::Mat camera_matrix, distortion_coefficients, rvec, tvec;

      void init(string IntrinsicFile, string ExtrinsicFile, double SCALE);

      //! Constructor
      CameraModel();
      //! Destructor
      virtual ~CameraModel();

      //! from image coordinate to world coordinate (single points)
      bool imageToWorld(double Xi, double Yi, double Zw, double& Xw,
          double &Yw);

      //! from world coordinate to image coordinate (single points)
      bool worldToImage(double Xw, double Yw, double Zw, double& Xi,
          double& Yi);

      //! from image coordinate to world coordinate (Matrix)
      bool imageToWorldMat(cv::Mat image_coordinates,
          cv::Mat& world_coordinates);

      //! from world coordinate to image coordinate (Matrix)
      bool worldToImageMat(cv::Mat world_coordinates,
          cv::Mat& image_coordinates);

      //! Loading from an XML
      virtual bool fromXml(string IntrinsicFile, string ExtrinsicFile,
          double SCALE);

  };
}
;

#endif

