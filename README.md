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
-  delay(1000) : permet de faire une pause d'une seconde
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
  inr interValue = digitalRead(interPin) // llit la valeur 

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

## des exemples : 

### faire clignoter une led 

- brancher une led entre la  broche **D1**  et la masse (gnd). Attention à la tension de sortie de la broche **D1**, adaptez la résistance à mettre en serie
- faire un programme
  - definir la broche **D1** en tant que sortie et en **LOW** dans la partie setup()
  - dans la partie loop(), mettre la valeur sur **D1** à **LOW**, attendre 1 seconde, mettre la valeur sur **D1** à **high**, attendre 1 seconde et on double
  - sur le terminal serie, afficher l'état de la led 

<spoiler>
 
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
</spoiler>


### lire la valeur d'une résistance variable 

### faire varier la vitesse de clignotement de la led en fonction de la résistance

### allumer une led quand on va appuyer sur un bouton 
<spoiler>
</spoiler>

### faire un chenillard 
<spoiler>
</spoiler>

### lire une sonde de température 'one wire' 

<spoiler>
</spoiler>


