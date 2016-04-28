#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Kinect2.h"
#include "Osc.h"
#include "cinder/gl/Texture.h"
#include "cinder/Text.h"

using namespace ci;
using namespace ci::app;
using namespace std;

const std::string destinationHost = "127.0.0.1";
const uint16_t destinationPort = 8000;

class ReadingBLApp : public App {
  public:
	  ReadingBLApp();

	void setup() override;
	void keyDown( KeyEvent event ) override;
	void update() override;
	void draw() override;
	void render();
private:
	Kinect2::BodyFrame mBodyFrame;
	ci::Channel8uRef mChannelBodyIndex;
	ci::Channel16uRef mChannelDepth;
	Kinect2::DeviceRef mDevice;

	boolean snapshot;

	osc::SenderUdp mSender;
	osc::ReceiverUdp mReceiver;
	
	gl::Texture2dRef		mTextTexture;
	TextLayout simple;
	vec2				mSize;
	Font				mFont;
	std::string				emotion;
};

ReadingBLApp::ReadingBLApp() : App(), mReceiver(9000), mSender(8000, destinationHost, destinationPort)
{
	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectBodyEventHandler([&](const Kinect2::BodyFrame frame) {
		mBodyFrame = frame;
	});
	mDevice->connectBodyIndexEventHandler([&](const Kinect2::BodyIndexFrame frame)
	{
		mChannelBodyIndex = frame.getChannel();
	});
	mDevice->connectDepthEventHandler([&](const Kinect2::DepthFrame frame)
	{
		mChannelDepth = frame.getChannel();
	});
}

void ReadingBLApp::setup()
{	
	snapshot = false;
	
	mSender.bind();
	mReceiver.bind();
	mReceiver.listen();
	mReceiver.setListener("/prediction",
		[&](const osc::Message &message) {
		std::string s = message[0].string();
		emotion = s;
		ci::app::console() << s << endl;
	});

	mFont = Font("Times New Roman", 32);
	mSize = vec2(200, 100);
	emotion = "Waiting...";
	string txt = emotion;
	simple.setFont(mFont);
	simple.setColor(Color(1, 0, 0.1f));
	simple.addLine(emotion);
	mTextTexture = gl::Texture2d::create(simple.render(true, false));
}

void ReadingBLApp::render() {
	
}

void ReadingBLApp::keyDown( KeyEvent event )
{
	char key = event.getChar();
	if (key == 'a') {
		snapshot = true;
	}
}

void ReadingBLApp::update()
{
}

void ReadingBLApp::draw()
{
	const gl::ScopedViewport scopedViewport(ivec2(0), getWindowSize());
	const gl::ScopedMatrices scopedMatrices;
	const gl::ScopedBlendAlpha scopedBlendAlpha;
	gl::setMatricesWindow(getWindowSize());
	gl::clear();
	gl::color(ColorAf::white());
	gl::disableDepthRead();
	gl::disableDepthWrite();

	if (mChannelDepth) {
		gl::enable(GL_TEXTURE_2D);
		const gl::TextureRef tex = gl::Texture::create(*Kinect2::channel16To8(mChannelDepth));
		gl::draw(tex, tex->getBounds(), Rectf(getWindowBounds()));
		gl::draw(mTextTexture);
	}

	if (mChannelBodyIndex) {
		gl::enable(GL_TEXTURE_2D);

		auto drawHand = [&](const Kinect2::Body::Hand& hand, const ivec2& pos) -> void
		{
			switch (hand.getState()) {
			case HandState_Closed:
				gl::color(ColorAf(1.0f, 0.0f, 0.0f, 0.5f));
				break;
			case HandState_Lasso:
				gl::color(ColorAf(0.0f, 0.0f, 1.0f, 0.5f));
				break;
			case HandState_Open:
				gl::color(ColorAf(0.0f, 1.0f, 0.0f, 0.5f));
				break;
			default:
				gl::color(ColorAf(0.0f, 0.0f, 0.0f, 0.0f));
				break;
			}
			gl::drawSolidCircle(pos, 30.0f, 32);
		};

		gl::pushMatrices();
		gl::scale(vec2(getWindowSize()) / vec2(mChannelBodyIndex->getSize()));
		gl::disable(GL_TEXTURE_2D);
		for (const Kinect2::Body &body : mBodyFrame.getBodies()) {
			if (body.isTracked()) {
				gl::color(ColorAf::white());
				osc::Message msg("/skeletal_data");
				for (const auto& joint : body.getJointMap()) {
					if (joint.second.getTrackingState() == TrackingState::TrackingState_Tracked) {
						vec2 pos(mDevice->mapCameraToDepth(joint.second.getPosition()));
						if (snapshot) {
							msg.append(to_string(joint.second.getPosition().x));
							msg.append(to_string(joint.second.getPosition().y));
							msg.append(to_string(joint.second.getPosition().z));
							msg.append(to_string(joint.second.getOrientation().w));
							msg.append(to_string(joint.second.getOrientation().x));
							msg.append(to_string(joint.second.getOrientation().y));
							msg.append(to_string(joint.second.getOrientation().z));
							// indicate data is being recorded to the csv file
							gl::color(0, 1, 0);
							gl::drawSolidCircle(vec2(100, 100), 20);
						}
						gl::drawSolidCircle(pos, 5.0f, 32);
						vec2 parent(mDevice->mapCameraToDepth(body.getJointMap().at(joint.second.getParentJoint()).getPosition()));
						gl::drawLine(pos, parent);
					}
					else {
						// warning signal
						gl::color(1, 0, 0);
						gl::drawSolidCircle(vec2(100, 100), 20);
						//msg.append("N/A,N/A,N/A,N/A,N/A,N/A,N/A,");
						msg.append("N/A");
					}
				}
				if (snapshot) {
					mSender.send(msg);
					//snapshot = false;
				}
				drawHand(body.getHandLeft(), mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HandLeft).getPosition()));
				drawHand(body.getHandRight(), mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HandRight).getPosition()));
			}
		}
	}
	
}

CINDER_APP( ReadingBLApp, RendererGl )
