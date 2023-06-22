#include"opengltranslate.h"

OpenCVOpenGLWidget::OpenCVOpenGLWidget(QWidget *parent) : QOpenGLWidget(parent)
{
    mBgColor = QColor::fromRgb(0,0,0);
}

void OpenCVOpenGLWidget::SetCharAvailable(bool available)
{
    this->is_char_available = available;
}

bool OpenCVOpenGLWidget::GetCharAvailable()
{
    return this->is_char_available;
}

QPoint OpenCVOpenGLWidget::getQPoint()
{
    return this->get_point;
}

void OpenCVOpenGLWidget::initializeGL()
{
    makeCurrent();
    initializeOpenGLFunctions();

    float r = ((float)mBgColor.darker().red()) / 255.0f;
    float g = ((float)mBgColor.darker().green()) / 255.0f;
    float b = ((float)mBgColor.darker().blue()) / 255.0f;

    glClearColor(r,g,b,0.0f);
}

void OpenCVOpenGLWidget::resizeGL(int width,int height)
{
    makeCurrent();
    glViewport(0,0,(GLint)width,(GLint)height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(0,width,-height,0,0,1);
    glMatrixMode(GL_MODELVIEW);

    recalculatePos();

    emit imageSizeChanged(mRenderWidth,mRenderHeight);
    updateScene();
}

void OpenCVOpenGLWidget::paintGL()
{
    makeCurrent();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderImage();
}

void OpenCVOpenGLWidget::updateScene()
{
    if(this->isVisible())
        update();
}

void OpenCVOpenGLWidget::renderImage()
{
    drawmutex.lock();
    makeCurrent();

    glClear(GL_COLOR_BUFFER_BIT);
    if(!mRenderQtImg.isNull())
    {
        if(mRenderWidth == mRenderQtImg.width() && mRenderHeight == mRenderQtImg.height())
            mResizedImg = mRenderQtImg;
        else
            mResizedImg = mRenderQtImg.scaled(QSize(mRenderWidth,mRenderHeight),Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
        
        glRasterPos2i(mRenderPosX,mRenderPosY);
        glPixelZoom(1,-1);
        glDrawPixels(mResizedImg.width(),mResizedImg.height(),GL_RGBA,GL_UNSIGNED_BYTE,mResizedImg.bits());
    }
    glPopMatrix();
    glFlush();
    drawmutex.unlock();
}

void OpenCVOpenGLWidget::recalculatePos()
{
    mImgRatio = (float)mOrigImage.cols / (float)mOrigImage.rows;
    mRenderWidth = this->size().width();
    mRenderHeight = floor(mRenderWidth / mImgRatio);

    if (mRenderHeight > this->size().height())
    {
        mRenderHeight = this->size().height();
        mRenderWidth = floor(mRenderHeight * mImgRatio);
    }

    mRenderPosX = floor((this->size().width() - mRenderWidth) / 2);
    mRenderPosY = -floor((this->size().height() - mRenderHeight) / 2);

    mResizedImg = QImage();
}

bool OpenCVOpenGLWidget::showImage(const cv::Mat& image)
{
    	drawmutex.lock();
    if (image.channels() == 3)
        cvtColor(image, mOrigImage, cv::COLOR_BGR2RGBA);
    else if (image.channels() == 1)
        cvtColor(image, mOrigImage, cv::COLOR_GRAY2RGBA);
	else if (image.channels() == 4)
		mOrigImage = image;
    else return false;

    mRenderQtImg = QImage((const unsigned char*)(mOrigImage.data),
                          mOrigImage.cols, mOrigImage.rows,
                          mOrigImage.step1(), QImage::Format_RGB32);
	
    recalculatePos();

    updateScene();
	drawmutex.unlock();
    return true;
}

void OpenCVOpenGLWidget::enterEvent(QEvent* event)
{
    if(event->type() == QEvent::Enter)
    {
        ROS_DEBUG("Mouse is in Map");
        this->is_char_available = true;
    }
    QOpenGLWidget::enterEvent(event);
}

void OpenCVOpenGLWidget::leaveEvent(QEvent* event)
{
    if(event->type() == QEvent::Leave)
    {
        ROS_DEBUG("Mouse is leave Map");
        this->is_char_available = false;
    }
    QOpenGLWidget::enterEvent(event);
}

void OpenCVOpenGLWidget::mousePressEvent(QMouseEvent* event)
{
    if(event->button() == Qt::LeftButton)
    {
        ROS_DEBUG("You clicked the chosen button");
        QPoint pos = event->pos();
        get_point = pos;
        Q_EMIT mousePressStart();
    }
    QOpenGLWidget::mousePressEvent(event);


}