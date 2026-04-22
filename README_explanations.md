# Kalman Filter

<!-- ![formula](https://images.squarespace-cdn.com/content/v1/5b2d76525cfd790c4a218093/1608502019200-SCOZXGEPES1W86052M39/linear_kalman_filter_math.png) -->

Filtre de kalman pour un plan 3D traitant des données issues d'un capteur IMU embarquant:
- un accéléromètre
- un gyroscope
- un GPS

Les mesures subissent un bruit blanc gaussien tel que:
$$Accelerometer: \mu = 0 , \sigma = 10^{-3}$$
$$Gyroscope: \mu = 0 , \sigma = 10^{-2}$$
$$GPS: \mu = 0 , \sigma = 10^{-1}$$

## Initialization

On appelle **état du système** le vecteur qui contient les variables d'intérêt que l'on souhaite estimer. 

Dans notre cas, nous allons estimer la **position** et la **vélocité** du système dans les trois dimensions de l'espace, soit un vecteur d'état de la forme: <br>
$x = [x, y, z, v_x, v_y, v_z]^T$. <br>
<img src="assets/state_vector_x.png" alt="state vector" width="200"/>

### Connaissance a priori
Le filtre de Kalman se base  sur une forte connaissance a priori du syteme. Nécessitant d'une initialisation minutieuse:
- $F$: la matrice de transition d'état, qui décrit comment l'état du système évolue dans un intervalle de temps $\Delta t$.: <br>
``` math
F = \begin{bmatrix}
1 & 0 & 0 & \Delta t & 0 & 0 \\\
0 & 1 & 0 & 0 & \Delta t & 0 \\\
0 & 0 & 1 & 0 & 0 & \Delta t \\\
0 & 0 & 0 & 1 & 0 & 0 \\\
0 & 0 & 0 & 0 & 1 & 0 \\\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
```
avec: $\Delta t = 0.01$


- $H$: Matrice d'observation. Relie les observations aux états du système. Dans notre cas on observe uniquement les positions:
``` math
H = \begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0
\end{bmatrix}
```

- $Q$: Matrice de covariance du bruit de filtre. Elle modélise l'incertitude de notre modèle de prédiction. Par exemple, notre modèle à vitesse constante ne prend pas en compte les accélérations. $Q$ représente la covariance de ce bruit de modèle (accélération aléatoire non modélisée).

- $R$: la matrice de covariance des mesures issue de l'IMU.
Définit par les specifications du capteur.
``` math
R = \begin{bmatrix}
\sigma_{gps}^2 & 0 & 0 \\
0 & \sigma_{gps}^2 & 0 \\
0 & 0 & \sigma_{gps}^2
\end{bmatrix}
```
Où: $\sigma_{gps}^2 = (10^{-1})^2 = 10^{-2}$


- $P$: Incertitude initiale sur l'estimation de l'état du système.
``` math
P = \begin{bmatrix}
\sigma_{pos}^2 & 0 & 0 & 0 & 0 & 0 \\
0 & \sigma_{pos}^2 & 0 & 0 & 0 & 0 \\
0 & 0 & \sigma_{pos}^2 & 0 & 0 & 0 \\
0 & 0 & 0 & \sigma_{vel}^2 & 0 & 0 \\
0 & 0 & 0 & 0 & \sigma_{vel}^2 & 0 \\
0 & 0 & 0 & 0 & 0 & \sigma_{vel}^2
\end{bmatrix} * \lambda
```
Où:

$\sigma_{vel}^2 = \sigma_{gyro}^2 + \sigma_{acc}^2 *(\Delta t)^2$ variance de la vélocité et 
    
$\lambda$ = 10 facteur de confiance dans l'estimation initiale. Définit de façon empirique.


![alt text](assets/kalman_covariance_matrix_p.png)


- $B$: la matrice de contrôle, qui relie les commandes (ici, l'accélération mesurée $u_k$) à l'état du système.
``` math
B = \begin{bmatrix}
0.5 * (\Delta t)^2 & 0 & 0 \\
0 & 0.5 * (\Delta t)^2 & 0 \\
0 & 0 & 0.5 * (\Delta t)^2 \\
\Delta t & 0 & 0 \\
0 & \Delta t & 0 \\
0 & 0 & \Delta t
\end{bmatrix}
```

- $x$: Etat initial du système. <br>
Au lancement de l'IMU, le capteur nous envoie la position et la vitesse réelle.
``` math
x = \begin{bmatrix}
x_0 \\
y_0 \\
z_0 \\
v_{x0} \\
v_{y0} \\
v_{z0}
\end{bmatrix}
```

Ces paramètres sont essentiels au fonctionnement du filtre et doivent être choisis avec soin en fonction du système à modéliser.

Ainsi, il est nécessaire d'avoir une bonne compréhension du système et de ses caractéristiques pour pouvoir initialiser correctement le filtre de Kalman.

D'une point de vue Bayesian, ce sont nos connaissances a priori.

## Prediction

1. Prédiction du prochain état à partir de l'état actuel et du contrôle d'entrée:
$ x_{k|k-1} = F x_{k-1|k-1} + B u_k $
Où:
- $x_{k|k-1}$: Estimation a priori de l'état du système à l'instant $k$ avant la mise à jour avec les mesures.
- $x_{k-1|k-1}$: Estimation a posteriori de l'état du système à l'instant $k-1$ après la mise à jour avec les mesures.
- $u_k$: Vecteur de commande (accélération mesurée par l'accéléromètre) à l'instant $k$. (optionnel)


2. Propagation de l'incertitude de l'estimation:<br>
$ P_{k|k-1} = F P_{k-1|k-1} F^T + Q $ <br><br>
La forme $F P_{k-1|k-1} F^T$ permet de calculer la covariance de $P_{k|k-1}$ en prenant en compte la transformation linéaire $F$. 
<br>
Car : $Cov [Ax + b] = ACov [x] A^T$.
<br><br>
Le terme $Q$ impacte directement la covariance de $P_{k|k-1}$ en ajoutant une incertitude supplémentaire provennat du bruit interne du modèle.

## Update

1. **Résidu de l'innovation**<br>
$ y_k = z_k - H x_{k|k-1} $<br>
Où:
- $y_k$: Résidu de l'innovation, qui représente la différence entre la mesure réelle $z_k$ et la mesure prédite $H x_{k|k-1}$. C'est une mesure de l'erreur de prédiction du système.
- $z_k$: Mesure réelle à l'instant $k$ (ici, la position GPS) .


2. **Covariance de l'innovation**<br>
$ S_k = H P_{k|k-1} H^T + R $<br>
Où:
- $S_k$: Covariance de l'innovation, qui représente l'incertitude totale (incertitude des prédictions + incertitude de la mesure).


3. **Gain de Kalman optimal**<br>
$ K_k = P_{k|k-1} H^T S_k^{-1} $<br>
Où:
- $K_k$: Gain de Kalman optimal, qui détermine notre niveau de confiance dans les mesures par rapport à nos prédictions.
<br><br>
gain élevé = confiance élevée dans les mesures.<br>
gain faible = confiance élevée dans les prédictions.


4. **Correction de l'estimation de l'état**<br>
$ x_{k|k} = x_{k|k-1} + K_k y_k $<br>
Où:
- $x_{k|k}$: Estimation a posteriori de l'état du système à l'instant $k$ après la mise à jour avec les mesures.

5. **Correction de l'incertitude d'estimation**<br>
$ P_{k|k} = (I - K_k H) P_{k|k-1} $<br>
Où:
- $P_{k|k}$: Incertitude a posteriori de l'estimation de l'état du système à l'instant $k$ après la mise à jour via les mesures.
- $I$: Matrice identité.

## Bibliography:
[https://medium.com/@sophiezhao_2990/kalman-filter-explained-simply-2b5672429205](https://medium.com/@sophiezhao_2990/kalman-filter-explained-simply-2b5672429205)

[engineeringmedia.com/controlblog/the-kalman-filter](https://engineeringmedia.com/controlblog/the-kalman-filter)

[https://wirelesspi.com/the-easiest-tutorial-on-kalman-filter/](https://wirelesspi.com/the-easiest-tutorial-on-kalman-filter/)

## Notes:

Revoir formule std de velocité.<br>
Revoir initialisation de Q.<br>
Facteur $\lambda$ multiplicatif de P.<br>