#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Kinect2.h"
#include "Osc.h"
#include "cinder/gl/Texture.h"
#include "cinder/Text.h"
#include "cinder/params/Params.h"
#include "../src/AntTweakBar/AntTweakBar.h"


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
private:
	Kinect2::BodyFrame mBodyFrame;
	ci::Channel8uRef mChannelBodyIndex;
	ci::Channel16uRef mChannelDepth;
	Kinect2::DeviceRef mDevice;

	bool snapshot;
	bool fullScreen;
	bool bodies;

	osc::SenderUdp mSender;
	osc::ReceiverUdp mReceiver;
	
	gl::Texture2dRef		mTextTexture;
	TextLayout simple;
	TextLayout simpleEmotion;
	vec2				mSize;
	Font				mFont;
	std::string				emotion;
	std::string tempEmotion, prevEmotion;
	int counter;
	Color clrRing;

	params::InterfaceGlRef	mParams;

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
	fullScreen = true;
	setFullScreen(fullScreen);

	snapshot = false;
	bodies = false;
	counter = 0;

	emotion = "";
	clrRing = ColorA::white();

	//mFont = Font("Times New Roman", 46);
	//mSize = vec2(250, 150);
	//emotion = "Waiting...";
	//simple.setFont(mFont);
	//simple.setColor(Color(1, 0, 0));
	//simple.addLine(emotion);
	//mTextTexture = gl::Texture2d::create(simple.render(true, false));

	mSender.bind();
	mReceiver.bind();
	mReceiver.listen();
	mReceiver.setListener("/prediction",
		[&](const osc::Message &message) {
		tempEmotion = message[0].string();
		if (tempEmotion != emotion && tempEmotion == prevEmotion) {
			counter++;
		}
		if (counter > 20) {
			counter = 0;
			if (tempEmotion == "bad data") {
				emotion = "Processing...";
			}
			else {
				emotion = tempEmotion;
			}
		}
		/*TextLayout simpleEmotion;
		simpleEmotion.setFont(mFont);
		simpleEmotion.setColor(Color(1, 0, 0));
		simpleEmotion.addLine(emotion);
		mTextTexture = gl::Texture2d::create(simpleEmotion.render(true, false));*/
		ci::app::console() << emotion << endl;
		prevEmotion = tempEmotion;
	});

	// Create the interface and give it a name.
	mParams = params::InterfaceGl::create(getWindow(), "Parameters", toPixels(ivec2(325, 200)));
	TwDefine(" GLOBAL fontsize=3 ");
	TwDefine(" GLOBAL fontscaling=2 ");
	TwDefine("Parameters valueswidth=150 "); // column width fits content
	mParams->addParam("Full Screen", &fullScreen).updateFn([this] {setFullScreen(fullScreen); });
	mParams->addParam("Analyze Body Language", &snapshot);
	mParams->addSeparator();
	mParams->addParam("Emotion: ", &emotion).updateFn([this] { console() << "new value: " << emotion << endl; });
	mParams->addSeparator();
	mParams->addParam("color", &clrRing, "opened=true");
	

}

void ReadingBLApp::keyDown( KeyEvent event )
{
	char key = event.getChar();
	if (key == 'a') {
		snapshot = true;
	}
	else if (key == 'q') {
		
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

	bodies = false;

	if (mChannelDepth) {
		gl::enable(GL_TEXTURE_2D);
		const gl::TextureRef tex = gl::Texture::create(*Kinect2::channel16To8(mChannelDepth));
		gl::draw(tex, tex->getBounds(), Rectf(getWindowBounds()));
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
				bodies = true;
				gl::color(clrRing);
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
						}
						gl::drawSolidCircle(pos, 5.0f, 32);
						vec2 parent(mDevice->mapCameraToDepth(body.getJointMap().at(joint.second.getParentJoint()).getPosition()));
						gl::drawLine(pos, parent);
					}
					else {
						emotion = "Processing...";
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

	if (!bodies) {
		emotion = "";
	}
	
	// Draw the interface
	mParams->draw();	
}

CINDER_APP( ReadingBLApp, RendererGl )
