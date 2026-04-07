# Φύλακας του Ληθαίου


Την πόλη των Τρικάλων την διαρρέει ένας όμορφος ποταμός, ο Ληθαίος.

Αυτός ο ποταμός σε φυσιολογικές καιρικές συνθήκες ρέει σταθερά και ομαλά.

Όμως σε ακραίες κακοκαιρίες (πχ κακοκαιρία Ntaniel) γίνεται βίαιος και καταστροφικός. 
Επίσης δεν καθαρίζεται σε τακτά χρονικά διαστήματα και η κοίτη του γεμίζει με αυτοφυή φύκια και βλάστηση, με αποτέλεσμα αυτά να συγκρατούν και άλλα σκουπίδια.

Έτσι σκεφτήκαμε να κατασκευάσουμε ένα ρομπότ με τις εξής δυνατότητες.
 Να κινείτε αυτόνομα και να καθαρίζει το ποτάμι κόβοντας την βλάστηση , να κινείται με ηλιακή ενέργεια, να μετράει θερμοκρασία, βροχόπτωση, αγωγιμότητα, βάθος και όλα αυτά τα στοιχεία να τα μεταδίδει μέσω wi fi σε ένα κεντρικό υπολογιστή ώστε να γνωρίζουν οι υπάλληλοι του Δήμου κάθε στιγμή την κατάστασή του και να παίρνουν έγκαιρα μέτρα προστασίας του πληθυσμού, επίσης θα είναι πάντα καθαρός.

# Υλικά

1     5V 4 Channel Relay Module with light coupling

4     9V Small DC Motor

1     40Pcs M/F Dupont Wire Jumper Cables 40 cm

1     40Pcs M/M Dupont Wire Jumper Cables

1     5V 1.5A Linear Voltage Regulator - 7805 TO-220

2     Acrylic Ultrasonic Sensor Mounting HC-SR04

2     ULTRASONIC SENSOR  HC-SR04 MODULE

1      Arduino MEGA 2560 Rev3 - (A000067)

1      Breadboard SYB-120 700 holes breadboard

2      DC Motor 12V 50RPM (JGA25-370)

1      Digital Temperature and Humidity Sensor

4      GL5516 Light Dependent Resistor LDR 5MM

1      LM317 Voltage Regulator LM317T

1      MPU-6050 3-Axis Gyroscope+Accelerometer

1      NodeMcu Lua CH340G ESP8266 WIFI

1      Development Board

1      Raindrops Detection and Humidity Sensor 

1      Pair 2.1x5.5mm Male & Female Barrel DC Power Jack

1      TDS Sensor Water Conductivity Sensor for Arduino Liquid Detection Water Quality

1       Voltage Sensor Module For Robot Arduino

1       JSN-SR04T Ultrasonic Distance Measuring Sensor – Waterproof

1       Waveshare Polysilicon Solar Panel (18V 10W), Photovoltaic Panel

2      SS-5GL2 Microswitch Hinge Roller Lever SPDT 3Pin Subminiature Basic Limit Switch

3      9V 2A LM7809

1      κόντρα πλακέ  30χ30 , βίδες, κόλα.

# ΚΑΤΑΣΚΕΥΗ

Ξεκινήσαμε την κατασκευή με την τοποθέτηση του arduino Mega και το Γυροσκόπιο πάνω στο κόντρα πλακέ

Προγραμματίζουμε το MPU6050 και κάνουμε δοκιμές καλής λειτουργίας

Κατόπιν ασχολούμαστε με έναν κάθε φορά αισθητήρα και δοκιμάζουμε την καλή του λειτουργία

Σχεδιάζουμε με χαρτόνι τα μηχανικά μέρη που πρέπει να κατασκευάσουμε

Κόβουμε  και δημιουργούμε αυτά τα μηχανικά μέρη

Δοκιμάζουμε την λειτουργικότητα των μηχανικών μερών και κάνουμε κάποιες βελτιώσεις

Δοκιμάζουμε την καλή αναμετάδοση του WIFI από το ESP8266 με ένα tablet 4G

Τοποθετούμε τα μοτέρ και κάνουμε πάλι δοκιμές 

Αφού όλα ανταποκρίνονται κανονικά είμαστε έτοιμοι για την τελική δοκιμή σε πραγματικές συνθήκες στο ποτάμι.

Φυσικά ο προγραμματισμός αυτού του πολύπλοκου ρομπότ επιδέχεται παρα πολλές ακόμη βελτιώσεις

Για τον προγραμματισμό του συμβουλευτήκαμε τις ΑΙ εφαρμογές, ChatGpt , DeepSeek, οι οποίες μας βοήθησαν πάρα πολύ για να λύσουμε πολλά προβλήματα που προέκυπταν.
