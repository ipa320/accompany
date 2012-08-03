#include "Background.hh"
#include <vnl/algo/vnl_symmetric_eigensystem.h>
#include <tools/string.hh>
#include "Helpers.hh"           // width, height, depth, ...
#include <data/XmlFile.hh>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

#define elt_prod element_product
#define elt_quot element_quotient


extern unsigned width, height;

void Background::processImage(const vnl_vector<FLOAT> &img)
{
	if (n==N) {
		// sum -= imgs.front();
		valImgs.push_back(img);
	} else {
		n++;
		imgs.push_back(img);
	}
}


// void Background::update()
// {
//      sum = imgs.front();
//      n = 0;
//      for (list< vnl_vector<FLOAT> >::const_iterator i = imgs.begin()++; i!=imgs.end(); ++i,++n)
//           sum += *i;     
//      mu = sum/n;

//      vnl_matrix<FLOAT>
//           D(n,sum.size());
//      unsigned index = 0;
//      for (list< vnl_vector<FLOAT> >::const_iterator i=imgs.begin();
//           i!=imgs.end(); ++i,++index) 
//           D.set_row(index, *i-mu);
//      vnl_matrix<FLOAT>
//           Dt = D.transpose();

//      // D = D * D.transpose();

//      // cout << format(D) << endl;

//      vnl_symmetric_eigensystem<FLOAT>
//           eig(D * Dt);

//      eigenvectors = vector< vnl_vector<FLOAT> >(d);
//      // for (unsigned i=0; i!=N; ++i)
//      //      cout << "Eigenvalue " << i << "= " << eig.get_eigenvalue(i) << endl;

//      for (unsigned i=0; i!=d; ++i) {
//           // cout << "Eigenvector: " << format(eig.get_eigenvector(i));
//           // cout << " -> " << format(D.transpose()*eig.get_eigenvector(i)) << endl;
//           eigenvectors[i] = (Dt * eig.get_eigenvector(N-i-1)).normalize();
//      }
// }
void Background::update()
{
	sum = imgs.front();
	n = 1;
	for (list< vnl_vector<FLOAT> >::const_iterator i = ++imgs.begin(); i!=imgs.end(); ++i,++n) {
		sum += *i;
	}

	mu = sum/n;

	vnl_matrix<FLOAT>
	D(sum.size(),n);
	unsigned index = 0;
	cout << "222222" << endl;
	for (list< vnl_vector<FLOAT> >::const_iterator i=imgs.begin();
			i!=imgs.end(); ++i,++index)
		D.set_column(index, *i-mu);

	cout << "111111" << endl;
	vnl_symmetric_eigensystem<FLOAT>
	eig(D.transpose() * D);

	eigenvectors = vector< vnl_vector<FLOAT> >(d);

	for (unsigned i=0; i!=d; ++i) {
		eigenvectors[i] = (D * eig.get_eigenvector(N-i-1)).normalize();
	}

	// Compute how the different channels typically vary from the projection.
	sumsq.set_size(mu.size());
	sumsq.fill(0.);
	for (list< vnl_vector<FLOAT> >::const_iterator i=valImgs.begin();
			i!=valImgs.end(); ++i,++index) {
		vnl_vector<FLOAT> bg;
		// cout << "Update: img size= " << i->size() << endl;
		getBackground(*i, bg);
		bg -= *i;
		sumsq += elt_prod(bg,bg);
	}
	var = sumsq/valImgs.size();

	// medianFilter(var, 5);
	var.fill(var.sum()/var.size());
	// vector<double> channels = vector<double>(3);
	// for (unsigned i=0; i!=var.size(); ++i)
	//      channels[i%3] += var[i];
	// channels[0] /= var.size() / 3;
	// channels[1] /= var.size() / 3;
	// channels[2] /= var.size() / 3;
	// for (unsigned i=0; i!=var.size(); ++i)
	//      var[i] = channels[i%3];
}

void Background::segment(const vnl_vector<FLOAT> &img,
		vnl_vector<FLOAT> &bg,
		vnl_vector<FLOAT> &fg,
		std::vector<int> &mask)
{
	if (n<N) {
		bg = img;
		fg = img;
		mask.resize(width*height);
		cerr << "n=" << n << ",N=" << N << endl;
	} else {
		vnl_vector<FLOAT>
		v = img - mu,
		proj(d);
		assert(mu.size() == img.size());
		bg = mu;
		for (unsigned i=0; i!=d; ++i) {
			proj(i) = inner_product(v,eigenvectors[i]);
			// cout << "Proj(" << i << ") = " << proj(i) << endl;
			bg += proj(i) * eigenvectors[i];
		}
		// fg = img - bg;
		fg.set_size(img.size());
		mask = vector<int>(img.size(),0);
#if COLOUR
		unsigned index = 0;
		for (unsigned i=0; i+2<img.size(); i+=3,++index) {
			if (bg(i)>255)   bg(i) = 255;   else if (bg(i)<0) bg(i) = 0;
			if (bg(i+1)>255) bg(i+1) = 255; else if (bg(i+1)<0) bg(i+1) = 0;
			if (bg(i+2)>255) bg(i+2) = 255; else if (bg(i+2)<0) bg(i+2) = 0;
			FLOAT
			tmp0 = img(i)-bg(i),
			tmp1 = img(i+1)-bg(i+1)/*,
                                             tmp2 = img(i+2)-bg(i+2)*/;
			if (fabs(tmp0)+fabs(tmp1)> 50) {
				fg(i) = img(i);
				fg(i+1) = img(i+1);
				fg(i+2) = img(i+2);
				mask[index] = 1;
			} else {
				fg(i) = 255;
				fg(i+1) = 255;
				fg(i+2) = 255;
			}
		}

#else
		for (unsigned i=0; i!=fg.size(); ++i) {
			if (bg(i)>255)
				bg(i) = 255;
			else if (bg(i)<0)
				bg(i) = 0;
			if (fabs(img(i)-bg(i))> 50) {
				mask[i] = true;
				fg(i) = img(i);
			} else
				fg(i) = 255;
		}
#endif  // COLOUR
	}
}


void Background::getBackground(const vnl_vector<FLOAT> &img,
		vnl_vector<FLOAT> &bg)
{
	if (n<N) {
		bg.set_size(width*height);
		cerr << "n=" << n << ",N=" << N << endl;
	} else {
		vnl_vector<FLOAT>
		v = img - mu;
		bg = mu;

		for (unsigned i=0; i!=d; ++i) {
			FLOAT
			proj = inner_product(v,eigenvectors[i]);
			bg += proj * eigenvectors[i];
		}
		// Threshold it.
		unsigned max = bg.size() - 3;
		for (unsigned i=0; i<max; ) {
			if (bg(i)>255) bg(i) = 255; else if (bg(i)<0) bg(i) = 0;
			++i;
			if (bg(i)>255) bg(i) = 255; else if (bg(i)<0) bg(i) = 0;
			++i;
			if (bg(i)>255) bg(i) = 255; else if (bg(i)<0) bg(i) = 0;
			++i;
		}
	}
}


void Background::getWeightedSqDiff(const vnl_vector<FLOAT> &img,
		vnl_vector<FLOAT> &diff)
{
	if (n<N) {
		diff.set_size(width*height);
		diff.fill(0);
		cerr << "WARNING: n=" << n << ",N=" << N << endl;
	} else {
		vnl_vector<FLOAT>
		v = img - mu,
		bg = mu;
		for (unsigned i=0; i!=d; ++i) {
			FLOAT
			proj = inner_product(v,eigenvectors[i]);
			bg += proj * eigenvectors[i];
		}
		// cout << "\nIMG =" << img << endl;
		// cout << "MU = " << mu << endl;
		// cout << "BG = " << bg << endl;
		bg -= img;
		// cout << "DIFF = " << bg << endl;
		diff = elt_quot(elt_prod(bg,bg),var);
	}
}


void Background::segment(const vnl_vector<FLOAT> &img,
		vector<int> &mask)
{
	if (n<N) {
		mask.resize(width*height);
		cerr << "n=" << n << ",N=" << N << endl;
	} else {
		vnl_vector<FLOAT>
		v = img - mu,
		proj(d),
		bg = mu;
		for (unsigned i=0; i!=d; ++i) {
			proj(i) = inner_product(v,eigenvectors[i]);
			bg += proj(i) * eigenvectors[i];
		}
		mask = vector<int>(img.size());
#if COLOUR
		unsigned index=0;
		for (unsigned i=0; i+2<img.size(); i+=3, index++) {
			if (bg(i)>255)   bg(i) = 255;   else if (bg(i)<0) bg(i) = 0;
			if (bg(i+1)>255) bg(i+1) = 255; else if (bg(i+1)<0) bg(i+1) = 0;
			if (bg(i+2)>255) bg(i+2) = 255; else if (bg(i+2)<0) bg(i+2) = 0;
			FLOAT
			tmp0 = img(i)-bg(i),
			tmp1 = img(i+1)-bg(i+1)
			//, tmp2 = img(i+2)-bg(i+2)
			;
			mask[index] = (fabs(tmp0)+fabs(tmp1)> 50);
		}

#else
		for (unsigned i=0; i!=fg.size(); ++i) {
			if (bg(i)>255)
				bg(i) = 255;
			else if (bg(i)<0)
				bg(i) = 0;
			mask[i] = (fabs(img(i)-bg(i))> 50);
		}
#endif  // COLOUR
	}
}


void Background::getProjection(const vnl_vector<FLOAT> &img,
		vnl_vector<FLOAT> &proj)
{
	proj.set_size(d);
	vnl_vector<FLOAT>
	v = img - mu;
	for (unsigned i=0; i!=d; ++i)
		proj(i) = inner_product(v,eigenvectors[i]);
}

void img2vec(const IplImage *img, vnl_vector<FLOAT> &v)
{
	if (width==0) {
		width = img->width;
		height = img->height;
		depth = img->depth;
		channels = img->nChannels;
	}
	v.set_size(img->width * img->height * img->nChannels);
	v.fill(0.0);

	unsigned char *src = (unsigned char *)img->imageData;
	unsigned idx=0;
	for (int i=0; i!=img->height; ++i) {
		unsigned char *s = src;
		for (unsigned j=0; j!=width; ++j)
			for (int k=0; k!=channels; ++k, ++idx, ++s)
				v[idx] = *s;
		src += img->widthStep;
	}
}

IplImage *vec2img(const vnl_vector<FLOAT> &v)
{
	IplImage
	*img = cvCreateImage(cvSize(width,height), depth, channels);

	unsigned char *src = (unsigned char *)img->imageData;
	memset(src, 0, img->imageSize);
	unsigned idx=0;
	for (unsigned i=0; i!=height; ++i) {
		unsigned char *s = src;
		for (unsigned j=0; j!=width; ++j)
			for (int k=0; k!=channels; ++k, ++idx, ++s)
				*s = v[idx];
		src += img->widthStep;
	}

	return img;
}

IplImage *bmp2img(const vnl_vector<FLOAT> &v)
{
	IplImage
	*img = cvCreateImage(cvSize(width,height), 8, 1);

	FLOAT
	mx = v.max_value(),
	mn = v.min_value();
	if (mx < 1.)               // Assume they're probs
		mx = 1.;
	if (mn > 0)
		mn=0;
	FLOAT
	scale = 255.0 / (mx-mn);
	unsigned char *src = (unsigned char *)img->imageData;
	memset(src, 0, img->imageSize);
	unsigned idx=0;
	for (unsigned i=0; i!=height; ++i) {
		unsigned char *s = src;
		for (unsigned j=0; j!=width; ++j, ++idx, ++s)
			*s = (v[idx] - mn) * scale;
		src += img->widthStep;
	}

	return img;
}

void showImg(const char *win, const vnl_vector<FLOAT> &v, bool convert)
{
	IplImage
	*img = vec2img(v);
	if (convert) {
		IplImage
		*cvt = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,3);
		cvCvtColor(img,cvt,TO_IMG_FMT);
		cvShowImage(win, cvt);
		cvReleaseImage(&cvt);
	} else
		cvShowImage(win, img);

	cvReleaseImage(&img);
}


void Background::xmlPack(XmlFile &f) const
{
	f.pack("mu", mu);
	f.pack("v",eigenvectors);
	f.pack("var",var);
	// cerr << "Eigenvectors.size() " << eigenvectors.size() << ", dim= " << mu.size() << " -- " << eigenvectors[0].size() << endl;
}

void Background::xmlUnpack(XmlFile &f) 
{
	f.unpack("mu", mu);
	f.unpack("v",eigenvectors);
	if (f.countChildren("var")) {
		f.unpack("var",var);
		// cout << "Variance = " << var(0) << endl;
	} else {
		var.set_size(mu.size());
		var.fill(1.0);
		cerr << "No variance found!" << endl;
	}
	d = eigenvectors.size();
	n = N;
	cerr << "Eigenvectors.size() " << eigenvectors.size() << ", dim= " << mu.size() << " -- " << eigenvectors[0].size() << endl;
}

void getBackground(const char *filename, Background &bg)
{
	if (matchesnc(filename, "*.xml")) {
		XmlReader
		rd(filename);
		rd.unpack("Background", bg);
	} else {
		vector<string>
		trainset;
		listImages(filename,trainset);
		for (unsigned i=0; i!=trainset.size(); ++i) {
			IplImage *src = loadImage(trainset[i].c_str());
			IplImage *img = cvCreateImage(cvGetSize(src),IPL_DEPTH_8U,3);
			cvCvtColor(src,img,TO_INT_FMT);
			cvReleaseImage(&src);

			vnl_vector<FLOAT>
			imgV;
			img2vec(img, imgV);
			bg.processImage(imgV);
			cvReleaseImage(&img);
		}
		bg.update();
	}
}

void getBackground(const char *filename, vector<Background> &bg, unsigned C, unsigned smooth)
{
	if (matchesnc(filename, "*.xml")) {
		XmlReader
		rd(filename);
		rd.unpack("Background", bg);
	} else {
		// cerr<< "BG must be trained beforehand." << endl;
		// exit(1);
		// // vector<string>
		// //      trainset;
		// // listImages(filename,trainset);
		// // for (unsigned i=0; i!=trainset.size(); ++i) {
		// //      IplImage
		// //           *src = loadImage(trainset[i].c_str()),
		// //           *img = cvCreateImage(cvGetSize(src),IPL_DEPTH_8U,3);
		// //      cvCvtColor(src,img,TO_INT_FMT);
		// //      cvReleaseImage(&src);

		// //      vnl_vector<FLOAT>
		// //           imgV;
		// //      img2vec(img, imgV);
		// //      bg.processImage(imgV);
		// //      cvReleaseImage(&img);
		// // }
		// // bg.update();

		cerr << "Listing images" << endl;
		vector< vector<string> >
		trainset;
		listImages(filename,trainset);
		cerr << "Done." << endl;
		unsigned
		N=(unsigned)(0.8*trainset.size());

		bg = vector<Background>(trainset[0].size(),Background(N,C));

		for (unsigned j=0; j!=trainset[0].size(); ++j) {
			for (unsigned i=0; i!=trainset.size(); ++i) {
				string filename = trainset[i][j];
				cerr << "Loading " << filename << endl;
				IplImage *src = loadImage(filename.c_str());
				IplImage *img = cvCreateImage(cvGetSize(src),IPL_DEPTH_8U,3);
				if (smooth != 0)
					cvSmooth(src,src,CV_GAUSSIAN,smooth,smooth);
				cvCvtColor(src,img,TO_INT_FMT);
				cvReleaseImage(&src);

				vnl_vector<FLOAT>
				imgV;
				img2vec(img, imgV);
				bg[j].processImage(imgV);
				cvReleaseImage(&img);
			}
			cerr << "Updating " << j << endl;
			bg[j].update();
			bg[j].dropImg();
			cerr << "Done." << endl;
		}
	}
}


