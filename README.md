# intro_arduino

Bonjour,  si vous souhaitez commencer à programmer un arduino, peu importe lequel, vous pouvez suivre ce qui est ci dessous. 

## explication des broches 
Les broches standards 
 - D1 à Dx: Utilisées pour diverses entrées/sorties numériques.
 - A1 à Ax: Utilisées pour les entrées analogiques (0 à 1023). Converti un signal analogique (0V à 5V) en une valeur numérique (0 à 1023).
 - VIN: Tension d'entrée pour alimenter l'Arduino à partir d'une source externe.
 - 5V: Sortie de tension à 5V.
 - 3.3V: Sortie de tension à 3.3V.
 - GND (Ground): Broches de masse.

quasiement toutes les broches ont ce qu'on nomme des "alternates".  C'est à dire que l'on peut définir d'autres protocoles de communication 
 - Serial
 - I2C
 - SPI
 - PWM

   

## quelques commandes 
- delay(1000) : permet de faire une pause d'une seconde
- pinMode(pin,mode) : permet de definir le role de la broche 
- digitalWrite(ledPin, HIGH) : permet de passer la broche ledPin à 1
- digitalRead(interPin) : permet de lire la broche interPin
- analogRead(potPin) : permet de lire une valeur
- Serial.begin(9600) : definir le port serie et sa vitesse
- Serial.print("Valeur : ") : affiche "Valeur : " sur le port serie mais ne retourne pas à la ligne 
- Serial.print("Valeur : ") : affiche "Valeur : " sur le port serie puis retourne à la ligne
- on retrouve les commandes standards du C  comme if , while, ...
  
  
## le squelette minimal et un exemple 
- au debut du programme
  - on peut avoir des includes pour étendre les fonctions de base, comme du SPI ou I2C
    - #include <Wire.h>
  - on retrouve les variables globales qui definissent les broches par exemple.
    - const int ledPin = 1; // definir ledPin sur la broche 1 ( D1 => digital )
    - const int resistancePin = A1; // definir resistancePin sur la broche A1 ( Analogique ) 
- setup() : Fonction de configuration qui s'exécute une seule fois au démarrage du programme. Utilisée pour initialiser les variables, les configurations de broches
  - dans la partie setup, on peut initialiser les variables et broches de la façon suivante 
    -  pinMode(pin, mode); // permet de fixer la fonction de la broche "pin" . On peut avoir la valeur INPUT pour lire une entrée ( comme un interrupteur), OUTPUT pour écrire une valeur sur la broche
- loop() : Fonction principale qui s'exécute en boucle après la fonction setup(). Utilisée pour faire fonctionner le code en continu
  - digitalWrite(ledPin, HIGH); //permet de passer la broche ledPin à 1
  - analogRead(potPin); permet de lire la valeur de potPin


```

// Déclarations des constantes et variables globales
const int ledPin = 1;  // Exemple : Pin pour la LED
const int resisPin = A1; // Exemple : Pin pour la résistance
const int interPin = 2;  // Exemple : Pin pour l'interrupteur

void setup() {
  // Configuration des pins et initialisation de la communication série
  pinMode(ledPin, OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(interPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Lecture des valeurs des capteurs
  int potValue = analogRead(resisPin); // lit la valeur de la résistance
  inr interValue = digitalRead(interPin) // lit la valeur 

  // Traitement des données lues
  int delayTime = potValue

  // Action sur les composants 
  digitalWrite(ledPin, HIGH);
  delay(delayTime);

  // Affichage des données sur le port série
  Serial.print("valeur de l'interrupteur  : ");
  Serial.println(interValue);

  delay(500);
}

```

## quelques notes
  - LOW : état de sortie ou d'entrée à 0
  - HIGH : état de sortie ou d'entrée à 1
  - pull up : permet de fixer  la valeur 1 sur une entrée
  - pull down : permet de fixer la valeur 0 sur une entrée
  - pour la lecture "digitale", on peut brancher la broche sur le +5 ( par défaut)  ou sur la masse mais à ce moment là, il faut mettre INPUT_PULLUP dans pinMode
  - nous sommes sur du C, donc on termine les commandes avec un point virgule et les blocks sont entre {}

## IDE Arduino 
 pour les petits programmes que vous allez réaliser, il va vous falloir un IDE. Le plus simple est de prendre celui fourni par Arduino mais vous pouvez prendre VS Code. 

 en fonction de la board, il faudra peut être l'importer et il en sera de même pour les librairies externes. 

 

## des exemples à réaliser: 

essayer de faire le circuit avec le programme et si besoin, regardez la solution 

### faire clignoter une led 

A réaliser : 
- brancher une led entre la  broche **D1**  et la masse (gnd). Attention à la tension de sortie de la broche **D1**, adaptez la résistance à mettre en serie
- faire un programme
  - definir les variables 
  - definir la broche **D1** en tant que sortie et en **LOW** dans la partie setup()
  - dans la partie loop(), mettre la valeur sur **D1** à **LOW**, attendre 1 seconde, mettre la valeur sur **D1** à **high**, attendre 1 seconde
  - sur le terminal serie, afficher l'état de la led 

solution :
<details>
 
 ```
const int ledPin = 1;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(ledPin, HIGH);
  Serial.println("LED allumée");
  delay(1000);
  digitalWrite(ledPin, LOW);
  Serial.println("LED éteinte");
  delay(1000);
```
</details>


### lire la valeur d'une résistance variable 
A réaliser : 
 - brancher une resistance variable entre le +5 et la masse. la broche **A1** se branche au milieu pour prendre la valeur variable
- faire un programme
  - definir les variables
  - definir la broche **A1** en tant qu'entrée dans la partie setup()
  - dans la partie loop(), récupérer la valeur de la résistance 
  - sur le terminal serie, afficher la valeur de la résistance 
    
solution :

<details>
 
 ```
 // Définir les variables globales
const int sensorPin = A1;  // Broche connectée à la résistance variable
int sensorValue = 0;       // Variable pour stocker la valeur lue

void setup() {
  // Initialisation de la communication série
  Serial.begin(9600);

  // Définir la broche A1 en tant qu'entrée
  pinMode(sensorPin, INPUT);
}

void loop() {
  // Lire la valeur de la résistance variable
  sensorValue = analogRead(sensorPin);

  // Afficher la valeur lue sur le terminal série
  Serial.print("Valeur de la résistance variable : ");
  Serial.println(sensorValue);

  // Pause de 500 millisecondes avant la prochaine lecture
  delay(500);
}

```

</details>

### faire varier la vitesse de clignotement de la led en fonction de la résistance

on va réaliser cela avec un potentiometre mais cela peut etre une CTN ( resistance variable à la température) ou encore une résistance variable à la lumiere 

A réaliser : 
 - brancher une resistance variable entre le +5 et la masse. la broche **A1** se branche au milieu pour prendre la valeur variable
 - brancher une led entre la  broche **D1**  et la masse (gnd). Attention à la tension de sortie de la broche **D1**, adaptez la résistance à mettre en serie
- faire un programme
  - definir les variables
  - definir la broche **A1** en tant qu'entrée et la broche **D1** en tant que sortie et en **LOW**  dans la partie setup()
  - dans la partie loop(), récupérer la valeur de la résistance pour qu'elle serve de temps à attendre pour le clignotement de la led. mettre la valeur sur **D1** à **LOW**, attendre le temps défini par la ersistance variable , mettre la valeur sur **D1** à **high**, attendre le temps défini par la ersistance variable.
  - sur le terminal serie, afficher la valeur de la résistance et l'état de la led 

solution :
<details>

 ```
// Définir les variables globales
const int sensorPin = A1;  // Broche connectée à la résistance variable
const int ledPin = 1;      // Broche connectée à la LED
int sensorValue = 0;       // Variable pour stocker la valeur lue
unsigned long delayTime = 0; // Variable pour stocker le délai basé sur la résistance

void setup() {
  // Initialisation de la communication série
  Serial.begin(9600);

  // Définir les broches A1 et D1 en tant qu'entrée et sortie respectivement
  pinMode(sensorPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  // Initialement, mettre D1 en LOW (LED éteinte)
}

void loop() {
  // Lire la valeur de la résistance variable
  sensorValue = analogRead(sensorPin);

  // Convertir la valeur lue en temps de délai
  delayTime = map(sensorValue, 0, 1023, 100, 1000);  // Convertir en millisecondes (ajuste les valeurs selon besoin)

  // Afficher la valeur de la résistance et l'état de la LED sur le terminal série
  Serial.print("Valeur de la résistance variable : ");
  Serial.print(sensorValue);
  Serial.print(" | Délai (ms) : ");
  Serial.print(delayTime);
  Serial.print(" | État de la LED : ");
  Serial.println(digitalRead(ledPin) == HIGH ? "Allumée" : "Éteinte");

  // Allumer la LED, attendre, puis l'éteindre
  digitalWrite(ledPin, HIGH);
  delay(delayTime);
  digitalWrite(ledPin, LOW);
  delay(delayTime);
}

```
</details>

### allumer une led quand on va appuyer sur un bouton 

A réaliser : 
 - brancher un bouton poussoir sur **D2** et le +5V.
 - brancher une led entre la  broche **D1**  et la masse (gnd). Attention à la tension de sortie de la broche **D1**, adaptez la résistance à mettre en serie
- faire un programme
  - definir les variables
  - definir la broche **D2** comme entrée numérique et la broche **D1** en tant que sortie et en **LOW** dans la partie setup()
  - dans la partie loop(), récupérer la valeur du bouton poussoir : si il, **D2**,  est **HIGH** mettre la valeur sur **D1** à **HIGH** et **D2** est **LOW**  mettre la valeur sur **D1** à **LOW**. Evitez les rebonds 
  - sur le terminal serie, afficher la valeur du bouton  et l'état de la led
 

solution :
<details>
 
```
// Définir les variables globales
const int buttonPin = 2;    // Broche connectée au bouton poussoir
const int ledPin = 1;       // Broche connectée à la LED
int buttonState = 0;        // Variable pour stocker l'état actuel du bouton poussoir
int lastButtonState = 0;    // Variable pour stocker l'état précédent du bouton poussoir
unsigned long lastDebounceTime = 0;  // Dernière fois où l'état a changé
unsigned long debounceDelay = 50;    // Délai de détection des rebonds (50 ms)

void setup() {
  // Initialisation de la communication série
  Serial.begin(9600);

  // Définir les broches D2 et D1 comme entrée et sortie respectivement
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  // Initialement, mettre D1 en LOW (LED éteinte)
}

void loop() {
  // Lire l'état du bouton poussoir
  int reading = digitalRead(buttonPin);

  // Vérifier si l'état du bouton a changé
  if (reading != buttonState) {
    buttonState = reading;

    // Mettre à jour la LED en fonction de l'état du bouton poussoir
    if (buttonState == HIGH) {
      digitalWrite(ledPin, HIGH);  // Allumer la LED
    } else {
      digitalWrite(ledPin, LOW);   // Éteindre la LED
    }

    // Afficher l'état du bouton poussoir et de la LED sur le terminal série
    Serial.print("État du bouton : ");
    Serial.print(buttonState);
    Serial.print(" | État de la LED : ");
    Serial.println(digitalRead(ledPin) == HIGH ? "Allumée" : "Éteinte");
  }

  // Mettre à jour l'état précédent du bouton poussoir
  lastButtonState = reading;
}


```
</details>

### faire un chenillard 

si pas n'avez pas assez de sortie digitale, mettez moins de leds 

A réaliser : 
 - brancher 7 leds  entre les broches **D1** à **D7** et la masse (gnd). Attention à la tension de sortie de la broche **D1**, adaptez la résistance à mettre en serie
 - brancher un interupteur sur **D8** et un autre sur **D9**
- faire un programme
  - definir les variables. le nombre de led doit pouvoir être changé la 
  - definir les broche **D8** et **D9**  comme entrée numérique et les broche **D1** à **D7**  en tant que sortie et en **LOW** dans la partie setup()
  - dans la partie loop(), faire une boucle de 1 à 7.  Quand la boucle est a 1, cela allume la led 1 et éteint les autres. puis quand c'est 2, cela allume la led 2 et éteint les autres... jusqu'a 7. Quand on appui sur **D8**, cela ralenti le défilement, quand on appui sur **D9**, cela accélère 
  - sur le terminal serie, afficher la valeur des boutons  et la position de la led 
 
solution :
<details>

```
// Définir les variables globales
const int numLeds = 7;      // Nombre de LEDs
const int ledPins[] = {1, 2, 3, 4, 5, 6, 7}; // Broches connectées aux LEDs
const int slowButtonPin = 8; // Broche connectée à l'interrupteur pour ralentir
const int fastButtonPin = 9; // Broche connectée à l'interrupteur pour accélérer
int slowButtonState = 0;     // État de l'interrupteur de ralentissement
int fastButtonState = 0;     // État de l'interrupteur d'accélération
int ledIndex = 0;            // Indice de la LED actuellement allumée
unsigned long delayTime = 500; // Temps d'attente initial en millisecondes

void setup() {
  // Initialisation de la communication série
  Serial.begin(9600);

  // Définir les broches des LEDs comme sorties et les mettre en LOW
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }

  // Définir les broches des interrupteurs comme entrées
  pinMode(slowButtonPin, INPUT);
  pinMode(fastButtonPin, INPUT);
}

void loop() {
  // Lire l'état des interrupteurs
  slowButtonState = digitalRead(slowButtonPin);
  fastButtonState = digitalRead(fastButtonPin);

  // Ajuster le temps d'attente en fonction de l'état des interrupteurs
  if (slowButtonState == HIGH) {
    delayTime = min(delayTime + 50, 2000); // Ralentir (ajuster la limite max si nécessaire)
  }
  if (fastButtonState == HIGH) {
    delayTime = max(delayTime - 50, 50);  // Accélérer (ajuster la limite min si nécessaire)
  }

  // Allumer la LED actuellement sélectionnée et éteindre les autres
  for (int i = 0; i < numLeds; i++) {
    if (i == ledIndex) {
      digitalWrite(ledPins[i], HIGH);
    } else {
      digitalWrite(ledPins[i], LOW);
    }
  }

  // Afficher l'état des boutons et la position de la LED sur le terminal série
  Serial.print("État du bouton ralentir : ");
  Serial.print(slowButtonState);
  Serial.print(" | État du bouton accélérer : ");
  Serial.print(fastButtonState);
  Serial.print(" | Position de la LED : ");
  Serial.println(ledIndex + 1);

  // Attendre avant de passer à la LED suivante
  delay(delayTime);

  // Passer à la LED suivante
  ledIndex = (ledIndex + 1) % numLeds;
}

```
</details>

### lire une sonde de température 'one wire' 

petite variante de la variation de la vitesse en fontion d'une température. Celle ci sera prise sur un capteur numerique le DS18b20. 
A réaliser : 
 - brancher la sonde ds18b20 selon le datasheet https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf . la patte **data** sera sur **D2**
 - brancher une led entre la  broche **D1**  et la masse (gnd). Attention à la tension de sortie de la broche **D1**, adaptez la résistance à mettre en serie
- faire un programme
  - definir les variables
  - definir la broche **D2** en tant qu'entrée et la broche **D1** en tant que sortie et en **LOW**  dans la partie setup()
  - dans la partie loop(), récupérer la valeur de la résistance pour qu'elle serve de temps à attendre pour le clignotement de la led. mettre la valeur sur **D1** à **LOW**, attendre le temps défini par la ersistance variable , mettre la valeur sur **D1** à **high**, attendre le temps défini par la ersistance variable.
  - sur le terminal serie, afficher la valeur de la résistance et l'état de la led 

  - 
solution :
<details>
 
```
#include <OneWire.h>
#include <DallasTemperature.h>

// Définir les variables globales
const int ledPin = 1;          // Broche connectée à la LED
const int sensorPin = 2;       // Broche connectée à la sonde DS18B20
float temperature = 0.0;       // Variable pour stocker la température lue
unsigned long delayTime = 0;   // Variable pour stocker le délai basé sur la température

// Configurer le protocole OneWire et la bibliothèque DallasTemperature
OneWire oneWire(sensorPin);
DallasTemperature sensors(&oneWire);

void setup() {
  // Initialiser la communication série
  Serial.begin(9600);

  // Définir les broches comme sorties ou entrées
  pinMode(sensorPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  // Initialement, mettre D1 en LOW (LED éteinte)

  // Initialiser le capteur DS18B20
  sensors.begin();
}

void loop() {
  // Demander la température au capteur
  sensors.requestTemperatures();
  
  // Lire la température en Celsius
  temperature = sensors.getTempCByIndex(0);

  // Convertir la température en temps de délai (par exemple : multiplier par 100 pour obtenir des millisecondes)
  delayTime = temperature * 100;

  // Afficher la température et l'état de la LED sur le terminal série
  Serial.print("Température : ");
  Serial.print(temperature);
  Serial.print(" °C | État de la LED : ");
  Serial.println(digitalRead(ledPin) == HIGH ? "Allumée" : "Éteinte");

  // Allumer la LED, attendre, puis l'éteindre
  digitalWrite(ledPin, HIGH);
  delay(delayTime);
  digitalWrite(ledPin, LOW);
  delay(delayTime);
}


```
</details>


