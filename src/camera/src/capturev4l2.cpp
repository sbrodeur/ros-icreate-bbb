/******************************************************************************
 *
 * Copyright (c) 2014, Simon Brodeur
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  - Neither the name of the NECOTIS research group nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <camera/capturev4l2.h>

using namespace std;

static int xioctl(int fd, int request, void *arg){
        int r;
        do r = ioctl (fd, request, arg);
        while (-1 == r && EINTR == errno);
        return r;
}

VideoCapture::VideoCapture (string devname, int width, int height, int framerate, int exposure, int focus, int gain, bool decodeEnabled) {
  _devname = devname;
  _width = width;
  _height = height;
  _exposure = exposure;
  _focus = focus;
  _gain = gain;
  _decodeEnabled = decodeEnabled;
  _framerate = framerate;

  _fd = open(_devname.c_str(), O_RDWR);
  if (_fd == -1){
	  perror("Opening video device");
  }
  printInfo();
  setParameters();
  initMmap();

  _decoder = NULL;
  if (_decodeEnabled){
	  _decoder = new VideoDecoder(_width, _height);
	  _decoder->initDecoder();
  }
}

VideoCapture::~VideoCapture () {
	close(_fd);
	if (_decoder){
		delete _decoder;
		_decoder = NULL;
	}
}

int VideoCapture::printInfo()
{
        struct v4l2_capability caps = {};
        if (-1 == xioctl(_fd, VIDIOC_QUERYCAP, &caps))
        {
                perror("Querying Capabilities");
                return 1;
        }

        printf( "Driver Caps:\n"
                "  Driver: \"%s\"\n"
                "  Card: \"%s\"\n"
                "  Bus: \"%s\"\n"
                "  Version: %d.%d\n"
                "  Capabilities: %08x\n",
                caps.driver,
                caps.card,
                caps.bus_info,
                (caps.version>>16)&&0xff,
                (caps.version>>24)&&0xff,
                caps.capabilities);


        struct v4l2_cropcap cropcap;
        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl (_fd, VIDIOC_CROPCAP, &cropcap))
        {
                perror("Querying Cropping Capabilities");
                return 1;
        }

        printf( "Camera Cropping:\n"
                "  Bounds: %dx%d+%d+%d\n"
                "  Default: %dx%d+%d+%d\n"
                "  Aspect: %d/%d\n",
                cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
                cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
                cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);

        int support_grbg10 = 0;

        struct v4l2_fmtdesc fmtdesc = {0};
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        char fourcc[5] = {0};
        char c, e;
        printf("  FMT : CE Desc\n--------------------\n");
        while (0 == xioctl(_fd, VIDIOC_ENUM_FMT, &fmtdesc))
        {
                strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
                if (fmtdesc.pixelformat == V4L2_PIX_FMT_SGRBG10)
                    support_grbg10 = 1;
                c = fmtdesc.flags & 1? 'C' : ' ';
                e = fmtdesc.flags & 2? 'E' : ' ';
                printf("  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
                fmtdesc.index++;
        }

        struct v4l2_format fmt;
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = _width;
        fmt.fmt.pix.height = _height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (-1 == xioctl(_fd, VIDIOC_S_FMT, &fmt))
        {
            perror("Couldn't set pixel format");
            //return 1;
        }

        struct v4l2_streamparm stream_params;
        memset(&stream_params, 0, sizeof(stream_params));
        stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(_fd, VIDIOC_G_PARM, &stream_params) < 0)
        {
        	perror("Couldn't get camera framerate");
        	//return 1;
        }
        stream_params.parm.capture.timeperframe.numerator = 1;
        //TODO : It seems we must multiply by a factor of 2 to get the right framerate!
        stream_params.parm.capture.timeperframe.denominator = 2 * _framerate;
        if (xioctl(_fd, VIDIOC_S_PARM, &stream_params) < 0)
        {
        	perror("Couldn't set camera framerate");
        	//return 1;
        }

        strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
        printf( "Selected Camera Mode:\n"
                "  Width: %d\n"
                "  Height: %d\n"
                "  PixFmt: %s\n"
                "  Field: %d\n",
                fmt.fmt.pix.width,
                fmt.fmt.pix.height,
                fourcc,
                fmt.fmt.pix.field);

        return 0;
}

int VideoCapture::setParameters()
{
        struct v4l2_format fmt;
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = _width;
        fmt.fmt.pix.height = _height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (-1 == xioctl(_fd, VIDIOC_S_FMT, &fmt))
        {
            perror("Couldn't set pixel format");
            //return 1;
        }

        struct v4l2_streamparm stream_params;
        memset(&stream_params, 0, sizeof(stream_params));
        stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(_fd, VIDIOC_G_PARM, &stream_params) < 0)
        {
        	perror("Couldn't get camera framerate");
        	//return 1;
        }
        stream_params.parm.capture.timeperframe.numerator = 1;
        //TODO : It seems we must multiply by a factor of 2 to get the right framerate!
        stream_params.parm.capture.timeperframe.denominator = 2 * _framerate;
        if (xioctl(_fd, VIDIOC_S_PARM, &stream_params) < 0)
        {
        	perror("Couldn't set camera framerate");
        	//return 1;
        }

        // Manual exposure control
        // See: https://linuxtv.org/downloads/v4l-dvb-apis/extended-controls.html
        struct v4l2_control stream_control_exposure;
        stream_control_exposure.id = V4L2_CID_EXPOSURE_AUTO;
        stream_control_exposure.value = V4L2_EXPOSURE_MANUAL;
        if(xioctl(_fd, VIDIOC_S_CTRL, &stream_control_exposure) != 0)
        {
        	perror("Couldn't set camera exposure to manual");
        	//return 1;
        }

        // Manual exposure absolute value
        struct v4l2_control stream_control_exposure_value;
        stream_control_exposure_value.id = V4L2_CID_EXPOSURE_ABSOLUTE;
        stream_control_exposure_value.value = _exposure;
        if(xioctl(_fd, VIDIOC_S_CTRL, &stream_control_exposure_value) != 0)
        {
        	perror("Couldn't set camera exposure absolute value");
        	//return 1;
        }

        // Manual gain control
		struct v4l2_control stream_control_gain;
		stream_control_gain.id = V4L2_CID_AUTOGAIN;
		stream_control_gain.value = false;
		if(xioctl(_fd, VIDIOC_S_CTRL, &stream_control_gain) != 0)
		{
			perror("Couldn't set camera gain to manual");
			//return 1;
		}

		// Manual gain absolute value
		struct v4l2_control stream_control_gain_value;
		stream_control_gain_value.id = V4L2_CID_GAIN;
		stream_control_gain_value.value = _gain;
		if(xioctl(_fd, VIDIOC_S_CTRL, &stream_control_gain_value) != 0)
		{
			perror("Couldn't set camera gain value");
			//return 1;
		}


        // Manual focus
		struct v4l2_control stream_control_focus;
        stream_control_focus.id = V4L2_CID_FOCUS_AUTO;
        stream_control_focus.value = false;
		if(xioctl(_fd, VIDIOC_S_CTRL, &stream_control_focus) != 0)
		{
			perror("Couldn't set autofocus to manual");
			//return 1;
		}

        // Manual focus absolute value
		struct v4l2_control stream_control_focus_value;
		stream_control_focus_value.id = V4L2_CID_FOCUS_ABSOLUTE;
		stream_control_focus_value.value = _focus;
		if(xioctl(_fd, VIDIOC_S_CTRL, &stream_control_focus_value) != 0)
		{
			perror("Couldn't set camera focus absolute value");
			//return 1;
		}

        return 0;
}

int VideoCapture::initMmap()
{
    struct v4l2_requestbuffers req = {0};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(_fd, VIDIOC_REQBUFS, &req))
    {
        perror("Requesting Buffer");
        return 1;
    }

    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(_fd, VIDIOC_QUERYBUF, &buf))
    {
        perror("Querying Buffer");
        return 1;
    }

    _buffer = (uint8_t*) mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, buf.m.offset);
    printf("Length: %d\nAddress: %p\n", buf.length, _buffer);

    if(-1 == xioctl(_fd, VIDIOC_STREAMON, &buf.type))
    {
        perror("Start Capture");
        return 1;
    }

    return 0;
}

std::vector<uint8_t> VideoCapture::grabFrame()
{
	std::vector<uint8_t> frame;

    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(_fd, VIDIOC_QBUF, &buf))
    {
        perror("Query Buffer");
        return frame;
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(_fd, &fds);
    struct timeval tv = {0};
    tv.tv_sec = 2;
    int r = select(_fd+1, &fds, NULL, NULL, &tv);
    if(-1 == r)
    {
        perror("Waiting for Frame");
        return frame;
    }

    if(-1 == xioctl(_fd, VIDIOC_DQBUF, &buf))
    {
        perror("Retrieving Frame");
        return frame;
    }
    //printf("Image Length: %d\n", buf.bytesused);

    if (buf.bytesused > 0){
    	if (_decodeEnabled){
    		//printf("Decoding...\n");
    		frame = _decoder->decodeBuffer(_buffer, buf.bytesused);
    	}else{
    		//printf("Pass-through...\n");
    		frame = std::vector<uint8_t>(_buffer, _buffer + buf.bytesused);
    	}
    }
    return frame;
}

VideoDecoder::VideoDecoder (int width, int height) {
  _width = width;
  _height = height;

  _transcoder = NULL;
  _swscontext = NULL;
  _decodedFrame = NULL;
  _decodedFrameRGB = NULL;

  initDecoder();
}

VideoDecoder::~VideoDecoder () {

	if (_transcoder){
	    avcodec_close(_transcoder);
	    av_free(_transcoder);
	    _transcoder = NULL;
	}
	if (_swscontext){
		sws_freeContext(_swscontext);
	}
	if (_decodedFrame){
		av_free(_decodedFrame);
		_decodedFrame = NULL;
	}
	if (_decodedFrameRGB){
	    av_free(_decodedFrameRGB);
	 	_decodedFrameRGB = NULL;
	}
}


int VideoDecoder::initDecoder(){

	av_register_all();

    av_init_packet(&_packet);
    AVCodec * codecDecode = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
    _transcoder = avcodec_alloc_context3(codecDecode);
	avcodec_get_context_defaults3(_transcoder, codecDecode);
	avcodec_open2(_transcoder, codecDecode, NULL);

	_transcoder->codec_id = AV_CODEC_ID_MJPEG;
	_transcoder->width = _width;
	_transcoder->height = _height;

	#if LIBAVCODEC_VERSION_MAJOR > 52
		_transcoder->pix_fmt = PIX_FMT_YUV422P;
		_transcoder->codec_type = AVMEDIA_TYPE_VIDEO;
	#endif

	_decodedFrame = avcodec_alloc_frame();
	PixelFormat  pFormat = PIX_FMT_YUV422P;
	_decodedFrameSize = avpicture_get_size(pFormat,_transcoder->width,_transcoder->height);
	uint8_t* buffer = (uint8_t *) av_malloc(_decodedFrameSize*sizeof(uint8_t));
	avpicture_fill((AVPicture *) _decodedFrame,buffer,pFormat,_transcoder->width,_transcoder->height);

	_decodedFrameRGB = avcodec_alloc_frame();
	pFormat = PIX_FMT_RGB24;
	avpicture_alloc((AVPicture *)_decodedFrameRGB, PIX_FMT_RGB24, _transcoder->width, _transcoder->height);
	_decodedFrameRGBSize = avpicture_get_size(pFormat,_transcoder->width,_transcoder->height) ;
	buffer = (uint8_t *) av_malloc(_decodedFrameRGBSize*sizeof(uint8_t));
	avpicture_fill((AVPicture *) _decodedFrameRGB,buffer,pFormat,_transcoder->width,_transcoder->height);

	_swscontext = sws_getContext(_transcoder->width, _transcoder->height, PIX_FMT_YUV422P,
			_transcoder->width, _transcoder->height, PIX_FMT_RGB24, SWS_FAST_BILINEAR, NULL, NULL, NULL);

	return 0;
}

std::vector<uint8_t> VideoDecoder::decodeBuffer(uint8_t* buffer, int size){

	std::vector<uint8_t> frame;

	// decode
	_packet.size = size;
	_packet.data = buffer;
	int frameFinished;
	avcodec_decode_video2(_transcoder, _decodedFrame, &frameFinished, &_packet);
	if (frameFinished){
		// transform
		sws_scale(_swscontext, _decodedFrame->data, _decodedFrame->linesize, 0, _height,
				_decodedFrameRGB->data, _decodedFrameRGB->linesize);

		frame.resize(_decodedFrameRGBSize);
		int size = avpicture_layout((AVPicture *)_decodedFrameRGB, PIX_FMT_RGB24, _transcoder->width, _transcoder->height, &frame[0], _decodedFrameRGBSize);
		if (size == _decodedFrameRGBSize){
    		frame = std::vector<uint8_t>(_decodedFrameRGB->data[0], _decodedFrameRGB->data[0] + _decodedFrameRGBSize);
		 }else{
				perror("Layout error");
			    frame.resize(0);
		 }
	}else{
		perror("Decoding MJPEG frame was unsuccessful\n");
	}
	return frame;
}

int VideoDecoder::getWidth(){
	return _width;
}

int VideoDecoder::getHeight(){
	return _height;
}
