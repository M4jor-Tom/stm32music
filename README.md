-------ATTRIBUT 'melody' DE LA STRUCTURE 'partition'

Dans le projet suivant, il faut coder en dure les mélodies avec des fonctions qui retournent les notes.

Comme ce n'est pas pratique d'écrire des fonctions pour décrire une mélodie, la tâche à été simplifiée.

--CREER UNE NOTE
Il faut écrire les notes comme ceci:
1La0.5  (cette note décrit un 'La' de l'octave '1' en appui pendant '0.5s')

Si vous écrivez la note comme ceci:
1La
Alors la première regEx la transformera en 1La1

Elle sera alors traitable par la 2e regEx qui la transformera en fonction 'timeNote' et sera décrite (pour le 1La0.5) par
timeNote(La, 1, 0.5, 0.1),
et aura une attente de 0.1s par défaut (dernier argument)

REGEX:
#^([0-9])(do|dod|re|mib|mi|fa|fad|sol|sold|la|sib|si)$#                         --en normal     [=>$1$21]
#^([0-9])(do|dod|re|mib|mi|fa|fad|sol|sold|la|sib|si) ?([0-9]+(\.[0-9]+)?)$#    --en timeNote   [=>timeNote($2, $1, $3, 0.1),]


--FAIRE UNE PAUSE DANS LA MELODIE
utilisez la fonction blankNote(float) et indiquer en argument un temps en secondes.

--FINIR LA MELODIE
utilisez la fonction lastNote() à la fin de CHAQUE mélodie pour la terminer.

---
-------ATTRIBUT 'readSpeed' DE LA STRUCTURE 'partition'
Fait que la fonction play(partition, int) lira n fois plus vite la partition.

-------FONCTION play(partition, int)
Premier argument: votre partition construite comme décrite ci-dessus
Second argument: instruments
0: Moteur seulement (Difficilement musical)
1: Buzzer seulement (Difficilement original)
2: Moteur et buzzer (Sympatique, testez donc)
