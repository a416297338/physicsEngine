#include "DemoBallsAndCube.hpp"

#include <format>
#include <iostream>

#include "AabbBox.hpp"
#include "Circle.hpp"
#include "Consts.hpp"
#include "FpsCounter.hpp"
#include "MathUtils.hpp"

#include "math/Vector2.hpp"

DemoBallsAndCube::DemoBallsAndCube()
	: _window(
	sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT),
	WINDOW_NAME,
	sf::Style::Default,
	sf::ContextSettings(0, 0, 8)
	)
{
	_windowSize = CAM_SIZE;
	_window.setView(DEFAULT_VIEW);
	//_window.setFramerateLimit(50);

	_impulseSolver = std::make_unique<stw::ImpulseSolver>();
	_smoothPositionSolver = std::make_unique<stw::SmoothPositionSolver>();
	_world.AddSolver(_impulseSolver.get());
	_world.AddSolver(_smoothPositionSolver.get());

	if (!_lModern.loadFromFile("data/lmodern.otf"))
	{
		std::cerr << "Could not load font\n";
	}

	_fpsText.setFont(_lModern);
	_fpsText.setCharacterSize(256);
	_fpsText.setFillColor(sf::Color::Black);
	_fpsText.setPosition(-CAM_WIDTH / 2.0f, -CAM_HEIGHT / 2.0f);
	_fpsText.setScale(0.01f, 0.01f);
}

void DemoBallsAndCube::Run()
{
	constexpr bool everyFrame = false;
	auto ground = std::make_unique<AabbBox>(
		_world,
		stw::Vector2(_windowSize.x * 0.6f, _windowSize.y * 0.6f),
		stw::Vector2(0.0f, _windowSize.y * -0.3f),
		false
		);
	ground->RigidBody()->SetIsKinematic(false);
	ground->RigidBody()->SetTakesGravity(false);
	ground->RigidBody()->SetMass(std::numeric_limits<float>::max());
	ground->RigidBody()->SetIsDynamic(false);
	ground->SetColor(sf::Color::Green);
	_entities.push_back(std::move(ground));
	
	sf::CircleShape shape(50.f);
	// set the shape color to green
	shape.setFillColor(sf::Color(100, 250, 50));
	std::cout << "Starting main loop\n";

	sf::Clock clock;
	int a = 0;
	bool isMousePressed = false;
	int numKeyboard = 4;
	while (_window.isOpen())
	{
		a++;
		float deltaTime = clock.restart().asSeconds();
		//deltaTime = 0.001f;
		sf::Event event{};
		while (_window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
			{
				_window.close();
			}
			else if (event.type == sf::Event::MouseButtonPressed)
			{
				isMousePressed = true;
			}
			else if (event.type == sf::Event::MouseButtonReleased)
			{
				isMousePressed = false;
			}
			else if (event.type == sf::Event::Resized)
			{
				_windowSize = {
					static_cast<float>(event.size.width) * VIEW_RATIO,
					static_cast<float>(event.size.height) * VIEW_RATIO
				};
				//sf::Vector2f center = _windowSize / 2.0f;
				_window.setView(sf::View({ 0,0 }, _windowSize));
			}
		}
		
		for(int num=0 ;num<=9;num++)
		{
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Key(num+sf::Keyboard::Num0)))
				numKeyboard = num;
		}
		if (isMousePressed)
		{
			if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left))
			{
				auto positionInt = sf::Mouse::getPosition(_window);
				auto positionFloat = _window.mapPixelToCoords(positionInt);
				auto physicsPos = SfmlPosToSpe(positionFloat);
				_entities.push_back(
					std::make_unique<Circle>(_world, 1.0f, physicsPos));
			}
			else if (sf::Mouse::isButtonPressed(sf::Mouse::Right)&& numKeyboard>=3)
			{
				auto positionInt = sf::Mouse::getPosition(_window);
				auto positionFloat = _window.mapPixelToCoords(positionInt);
				auto physicsPos = SfmlPosToSpe(positionFloat);
				_entities.push_back(
					//std::make_unique<AabbBox>(_world, stw::Vector2(2.f,2.f), stw::Vector2(0,10), true));
					std::make_unique<ConvexBox>(_world, std::sqrt(2.0f), numKeyboard, physicsPos, true));
			}

			if constexpr (!everyFrame)
				isMousePressed = false;
		}

		// Step the physics
		
		_world.Step(deltaTime);

		// Update the entities
		for (const auto& entity : _entities)
		{
			entity->Update(deltaTime);
		}

		_fpsText.setString(std::to_string(a));

		// Clear the window
		_window.clear(sf::Color::White);
		// Render all the entities
		for (const auto& entity : _entities)
		{
			_window.draw(*entity);
		}
		_window.draw(_fpsText);

		_window.display();
		//_sleep(60);
	}
}
