/*
 * EDUBOT HELLO WORLD EXAMPLE
 * @Author: Maik Basso <maik@maikbasso.com.br>
*/

#include <algorithm>
#include <iostream>
#include "libs/EdubotLib.hpp"

int main(){

	std::vector<double> sonars(7);
	
	EdubotLib *edubotLib = new EdubotLib();

	//try to connect on robot
	if(edubotLib->connect()){

		// shows edubot sensors
		int showSensorsTimes = 10;
		
		while (showSensorsTimes > 0) {
			// sonars
			for (int i=0; i<7; i++) {
				std::cout << "S" << i << ": " << edubotLib->getSonar(i) << "m, ";
				sonars.at(i) = edubotLib->getSonar(i);
			}

			int indice = std::distance(sonars.begin(), std::max_element(sonars.begin(), sonars.end()));

			int angulo = (indice - 3) * 30;

			std::cout << "indice: " << indice << "| angulo: " << angulo << std::endl;

			auto maxDistance = std::max_element(sonars.begin(), sonars.end());
			double maxValue = *maxDistance;
			edubotLib->rotate(angulo);

			edubotLib->sleepMilliseconds(20 * abs(angulo));

			edubotLib->move(maxValue * 0.3);

			edubotLib->sleepMilliseconds(maxValue * 100);
			// bumpers
			// Verifica colisão com bumpers
			bool colisao = false;
			for (int i=0; i<4; i++) {
				bool bateu = edubotLib->getBumper(i);
				std::cout << "B" << i << ": " << (bateu ? "true":"false") << ", ";
				if (bateu) colisao = true;
			}
			
			if (colisao) {
				std::cout << ">> Colisão detectada! Executando manobra de evasão..." << std::endl;
			
				// Dá ré por 0.5 segundos
				edubotLib->move(-0.3);
				edubotLib->sleepMilliseconds(500);
			
				// Para os motores
				edubotLib->stop();
				edubotLib->sleepMilliseconds(200);
			
				// Rotaciona 90 graus para a direita (poderia alternar depois se quiser)
				edubotLib->rotate(90);
				edubotLib->sleepMilliseconds(2000);
			
				// Atualiza leituras dos sonares após manobra
				for (int i=0; i<7; i++) {
					sonars.at(i) = edubotLib->getSonar(i);
				}
			}

			std::cout << "leftcount: " << edubotLib->getEncoderCountLeft() << ", ";
			std::cout << "rightcount: " << edubotLib->getEncoderCountRight() << ", ";
			std::cout << "dt(looptime): " << edubotLib->getEncoderCountDT() << ", ";
			
			std::cout << "x: " << edubotLib->getX() << ", ";
			std::cout << "y: " << edubotLib->getY() << ", ";
			std::cout << "theta: " << edubotLib->getTheta() << ", ";
			
			std::cout << "battCell0: " << edubotLib->getBatteryCellVoltage(0) << ", ";
			std::cout << "battCell1: " << edubotLib->getBatteryCellVoltage(1) << ", ";
			std::cout << "battCell2: " << edubotLib->getBatteryCellVoltage(2);

			// line break
			std::cout << std::endl;

			showSensorsTimes--;

		}

		edubotLib->neutral();
		// Waits for two seconds for processes to reflect on the robot
		edubotLib->sleepMilliseconds(2000);

		/*
		 * Function stop
		 * Stop the motors / break
		*/
		edubotLib->stop();
		// Waits for two seconds for processes to reflect on the robot
		edubotLib->sleepMilliseconds(2000);

		/*
		 * Function disconnect
		 * disconnect from robot
		 * Important to user EVER
		*/
		edubotLib->disconnect();
	}
	else{
		std::cout << "Could not connect on robot!" << std::endl;
	}

	return 0;
}