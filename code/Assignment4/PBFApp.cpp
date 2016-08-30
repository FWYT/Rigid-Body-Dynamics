#include <GUILib/GLUtils.h>
#include "PBFApp.h"
#include "Constants.h"

// Ignore the fact that this is the rigid body assignment,
// the name of the file is PBFApp,
// and the class itself is named MassSpringApp...

MassSpringApp::MassSpringApp() {
	setWindowTitle("Assignment 4: Rigid Body Dynamics");
	TwAddSeparator(mainMenuBar, "sep2", "");

	TwAddVarRW(mainMenuBar, "Draw Particles", TW_TYPE_BOOLCPP, &ParticleSystem::drawParticles, "");
	TwAddVarRW(mainMenuBar, "Enable Gravity", TW_TYPE_BOOLCPP, &ParticleSystem::enableGravity, "");

	showGroundPlane = true;
	showDesignEnvironmentBox = true;
	showReflections = false;

	cubeVertShader = new GLShader();
	cubeFragShader = new GLShader();
	cubeVertShader->loadFromFile("lambertian.vert", GL_VERTEX_SHADER);
	cubeFragShader->loadFromFile("lambertian.frag", GL_FRAGMENT_SHADER);
	cubeShader = new GLShaderProgram();
	cubeShader->load(cubeVertShader, cubeFragShader);
	cubeMaterial = new GLShaderMaterial();
	cubeMaterial->setShaderProgram(cubeShader);

	particleSystem = new ParticleSystem(cubeMaterial);

	pickedParticle = -1;
	ready = true;
}

MassSpringApp::~MassSpringApp(void){
	delete particleSystem;
	delete cubeMaterial;
	delete cubeShader;
	delete cubeVertShader;
	delete cubeFragShader;
}

const double PICK_DISTANCE = 0.1;

//triggered when mouse moves
bool MassSpringApp::onMouseMoveEvent(double xPos, double yPos) {

	if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;

	return false;
}

//triggered when mouse buttons are pressed
bool MassSpringApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {

	if (button == 0) {
		// Left mouse
		if (action == 1) {
			// Down
		}
		else {
			// Up
			pickedParticle = -1;
		}
	}

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) {
		return true;
	}

	return false;
}

//triggered when using the mouse wheel
bool MassSpringApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool MassSpringApp::onKeyEvent(int key, int action, int mods) {
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	if (action == 1) {
		if (key == 'G') {
			if (ParticleSystem::enableGravity) {
				Logger::consolePrint("Disabled gravity");
				ParticleSystem::enableGravity = false;
			}
			else {
				Logger::consolePrint("Enabled gravity");
				ParticleSystem::enableGravity = true;
			}
		}
		if (key == 'R') {
			if (particleSystem->applyingRandomForces) {
				particleSystem->stopVacuumForces();
			}
			else {
				particleSystem->startVacuumForces();
			}
		}
	}

	return false;
}

bool MassSpringApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;

	return false;
}


void MassSpringApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);
}

void MassSpringApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}


// Run the App tasks
void MassSpringApp::process() {
	// Take enough steps so that we are always running in (close to) real time
	int numSteps = (int)((1. / 30.) / DELTA_T);
	if (numSteps < 1) numSteps = 1;
	for (int i = 0; i < numSteps; i++) {
		particleSystem->integrate_PBF(DELTA_T);
	}
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void MassSpringApp::drawScene() {
	particleSystem->drawParticleSystem();
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void MassSpringApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void MassSpringApp::restart() {

}

bool MassSpringApp::processCommandLine(const std::string& cmdLine) {

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

