#include <algorithm>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <map>
#include <vector>
#include "libs/EdubotLib.hpp"

int main() {
    std::vector<double> sonars(7);
    EdubotLib *edubotLib = new EdubotLib();

    std::srand(std::time(nullptr)); // seed para aleatoriedade

    if (edubotLib->connect()) {
        // Variáveis de controle
        double ultimaVelocidade = 0.0;
        int contadorRepeticoesSemMovimento = 0;
        const double LIMIAR_MOVIMENTO = 0.01;

        static double anguloSuavizado = 0;
        const double ALFA = 0.3;
        const double TOLERANCIA_SONAR = 0.05;

        std::map<std::pair<int, int>, int> posicoesVisitadas;

        while (edubotLib->getX() <= 2.75) {
            // Atualiza leituras
            for (int i = 0; i < 7; i++) {
                sonars[i] = edubotLib->getSonar(i);
            }

            // Marca posição atual como visitada
            double x = edubotLib->getX();
            double y = edubotLib->getY();
            int xi = static_cast<int>(x * 10);
            int yi = static_cast<int>(y * 10);
            std::pair<int, int> posAtual(xi, yi);
            posicoesVisitadas[posAtual]++;

            // Penaliza sonares que apontam para regiões já visitadas
            for (int i = 0; i < 7; i++) {
                double anguloSensor = (i - 3) * 30;
                double alcance = sonars[i];
                double dx = alcance * cos((edubotLib->getTheta() + anguloSensor) * M_PI / 180.0);
                double dy = alcance * sin((edubotLib->getTheta() + anguloSensor) * M_PI / 180.0);
                int xt = static_cast<int>((x + dx) * 10);
                int yt = static_cast<int>((y + dy) * 10);
                std::pair<int, int> posAlvo(xt, yt);
                if (posicoesVisitadas[posAlvo] > 0) {
                    sonars[i] *= 0.6; // penaliza região já visitada
                }
            }

            // Escolhe melhor direção
            double maiorDistancia = *std::max_element(sonars.begin(), sonars.end());
            std::vector<int> candidatos;
            for (int i = 0; i < 7; i++) {
                if (fabs(sonars[i] - maiorDistancia) < TOLERANCIA_SONAR) {
                    candidatos.push_back(i);
                }
            }

            // Prioriza frontal
            int melhorIndice = candidatos[0];
            int menorDesvio = abs(candidatos[0] - 3);
            for (int i = 1; i < candidatos.size(); ++i) {
                int desvio = abs(candidatos[i] - 3);
                if (desvio < menorDesvio) {
                    melhorIndice = candidatos[i];
                    menorDesvio = desvio;
                }
            }

            int anguloNovo = (melhorIndice - 3) * 30;
            anguloSuavizado = ALFA * anguloNovo + (1 - ALFA) * anguloSuavizado;

            // Aplica rotação suavizada
            edubotLib->rotate(static_cast<int>(anguloSuavizado));
            edubotLib->sleepMilliseconds(20 * abs(anguloSuavizado));

            // Salva posição antes do movimento
            double xAntes = edubotLib->getX();
            double yAntes = edubotLib->getY();
            double distanciaMovimento = maiorDistancia * 0.3;

            edubotLib->move(distanciaMovimento);
            edubotLib->sleepMilliseconds(maiorDistancia * 100);

            // Verifica deslocamento real
            double xDepois = edubotLib->getX();
            double yDepois = edubotLib->getY();
            bool deslocou = (fabs(xDepois - xAntes) > LIMIAR_MOVIMENTO || fabs(yDepois - yAntes) > LIMIAR_MOVIMENTO);

            if (!deslocou && fabs(distanciaMovimento - ultimaVelocidade) < 0.0001) {
                contadorRepeticoesSemMovimento++;
            } else {
                contadorRepeticoesSemMovimento = 0;
            }
            ultimaVelocidade = distanciaMovimento;

            // Executa manobra se empacado
            if (contadorRepeticoesSemMovimento >= 3) {
                std::cout << ">> Empacamento detectado. Executando manobra..." << std::endl;
                edubotLib->move(-0.3);
                edubotLib->sleepMilliseconds(500);
                edubotLib->stop();
                edubotLib->sleepMilliseconds(200);
                int anguloAleatorio = (std::rand() % 91 + 30);
                if (std::rand() % 2 == 0) anguloAleatorio *= -1;
                edubotLib->rotate(anguloAleatorio);
                edubotLib->sleepMilliseconds(20 * abs(anguloAleatorio));
                edubotLib->move(0.1);
                edubotLib->sleepMilliseconds(500);
                contadorRepeticoesSemMovimento = 0;
            }

            // Colisão com bumpers
            bool colisao = false;
            bool esquerda = false, direita = false;
            for (int i = 0; i < 4; i++) {
                if (edubotLib->getBumper(i)) {
                    colisao = true;
                    if (i == 0 || i == 2) esquerda = true;
                    if (i == 1 || i == 3) direita = true;
                }
            }

            if (colisao) {
                edubotLib->move(-0.3);
                edubotLib->sleepMilliseconds(500);
                edubotLib->stop();
                edubotLib->sleepMilliseconds(200);
                int direcaoGiro = -90;
                if (direita && !esquerda) direcaoGiro = -90;
                else if (esquerda && !direita) direcaoGiro = 90;
                edubotLib->rotate(direcaoGiro);
                edubotLib->sleepMilliseconds(2000);
                edubotLib->move(0.1);
                edubotLib->sleepMilliseconds(500);
                contadorRepeticoesSemMovimento = 0;
            }

            std::cout << "x: " << edubotLib->getX()
                      << ", y: " << edubotLib->getY()
                      << ", theta: " << edubotLib->getTheta() << std::endl;
        }

        // Finalização
        edubotLib->neutral();
        edubotLib->sleepMilliseconds(2000);
        edubotLib->stop();
        edubotLib->sleepMilliseconds(2000);
        edubotLib->disconnect();
    } else {
        std::cout << "Não foi possível conectar ao robô!" << std::endl;
    }

    return 0;
}
