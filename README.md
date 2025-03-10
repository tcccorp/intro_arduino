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
- digitalWrite(ledPin, HIGH) : permet de passer la broche ledPin à 1
- analogRead(potPin) : permet de lire une valeur
- Serial.begin(9600) : definir le port serie et sa vitesse
- Serial.print("Valeur : ") : affiche "Valeur : " sur le port serie mais ne retourne pas à la ligne 
- Serial.print("Valeur : ") : affiche "Valeur : " sur le port serie puis retourne à la ligne
  
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
  - definir la broche **A1** et la broche **D1** en tant que sortie et en **LOW** en tant qu'entrée dans la partie setup()
  - dans la partie loop(), récupérer la valeur de la résistance pour qu'elle serve de temps à attendre pour le clignotement de la led. mettre la valeur sur **D1** à **LOW**, attendre le temps défini par la ersistance variable , mettre la valeur sur **D1** à **high**, attendre le temps défini par la ersistance variable.
  - sur le terminal serie, afficher la valeur de la résistance et l'état de la led 
solution :
<details>
 
```

```
</details>

### faire un chenillard 

solution :
<details>

```

```
</details>

### lire une sonde de température 'one wire' 

solution :
<details>
```

```
</details>


