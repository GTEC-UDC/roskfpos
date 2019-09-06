#include "MLLocation.h"


MLLocation::MLLocation()
{

}

MLLocation::MLLocation(const Vector3& relPosT0, const Vector3& relPosT1){
    relativePosTO = relPosT0;
    relativePosT1 = relPosT1;
}



/**
 * Distancia de un punto dado a una lista de balizas.
 */
std::vector<double> MLLocation::distanceToBeacons(const Vector3& position,
                                          const std::vector<RangingMeasurement>& rangingMeasurements) const {
    std::vector<double> distances;

    for (int i = 0; i < rangingMeasurements.size(); ++i)
    {
        auto& r = rangingMeasurements[i];
        double distance = sqrt((r.beacon.position.x - position.x)*(r.beacon.position.x - position.x) +
                               (r.beacon.position.y - position.y)*(r.beacon.position.y - position.y) +
                               (r.beacon.position.z - position.z)*(r.beacon.position.z - position.z));
        distances.push_back(distance);
    }
    return distances;
}



Vector3 MLLocation::estimatePosition2D(const std::vector<RangingMeasurement>& rangingMeasurements,
                        const Vector3& previousEstimation) {
//    if (rangings.size() < 3) {
//        Vector3 position = { NAN, NAN, NAN };
//        return position;
//    }

//   ROS_INFO("MLLocation start estimatmePosition2D"); 
    int numMeasurements = rangingMeasurements.size();     
    if (numMeasurements<3){
        Vector3 position;
        position.x = NAN;
        position.y = NAN;
        position.z = NAN;
        return position;
    }
    Vector3 position = previousEstimation;
    double cost = 1e20, newCost = 1;
 
    double step = 1;
    Vector3 tentativePos;


    newCost = estimationError(rangingMeasurements, position);

    //ROS_INFO("MLLocation before while"); 
    while((std::abs(cost-newCost)/cost > 1e-3) && (newCost> 1e-6)) {
        
        cost = newCost;

        std::vector<double> distances = distanceToBeacons(position, rangingMeasurements);
        arma::vec gradient(2, arma::fill::zeros);
        arma::mat secondDeriv(2, 2, arma::fill::zeros);
            for (int i = 0; i < rangingMeasurements.size(); ++i)
    {
        auto& r = rangingMeasurements[i];
            gradient(0) += (r.ranging - distances[i])*
                     (r.beacon.position.x - position.x)/(distances[i]*r.errorEstimation);
            gradient(1) += (r.ranging - distances[i])*
                     (r.beacon.position.y - position.y)/(distances[i]*r.errorEstimation);
 
            double d3 = distances[i]*distances[i]*distances[i];
            secondDeriv(0, 0) += ( 1 - r.ranging/distances[i]
                                     + r.ranging*(r.beacon.position.x - position.x)*(r.beacon.position.x - position.x)/d3
                                )/r.errorEstimation;
            secondDeriv(1, 1) += ( 1 - r.ranging/distances[i]
                                     + r.ranging*(r.beacon.position.y - position.y)*(r.beacon.position.y - position.y)/d3
                                )/r.errorEstimation;
            double diffXY = r.ranging*(r.beacon.position.x - position.x)*(r.beacon.position.y - position.y)/(d3*r.errorEstimation);
            secondDeriv(0, 1) += diffXY;
            secondDeriv(1, 0) += diffXY;
        }
        arma::vec pos(2);
        pos(0) = position.x;
        pos(1) = position.y;
        arma::vec rhs = secondDeriv*pos - gradient*step;
        arma::vec newPos = arma::solve(secondDeriv, rhs);

        tentativePos.x = newPos(0);
        tentativePos.y = newPos(1);



 //        arma::vec shift = inv(secondDeriv)*gradient;
 //        position.x = position.x - shift(0);
 //        position.y = position.y - shift(1);
 //        position.z = position.z - shift(2);
        distances = distanceToBeacons(tentativePos, rangingMeasurements);
        double tentativeCost = estimationError(rangingMeasurements, tentativePos);
        
        //ROS_INFO("MLLocation tentativeCost: %f", tentativeCost); 
        
        //ROS_INFO("MLLocation step: %f", step);

        if (tentativeCost>cost){
            step /=2;
            //ROS_INFO("MLLocation step: %f", step);

        } else {
            newCost = tentativeCost;
            step = 1;
            position.x = newPos(0);
            position.y = newPos(1);
        }



        // for (const auto& r : rangingMeasurements) {
        //     newCost += (r.rangings-distances[i])*(r.ranging-distances[i])/r.errorEstimation;
        //     i++;
        // }
        //std::cout << newCost << std::endl;
         //ROS_INFO("MLLocation position.x: %f", position.x);
         //ROS_INFO("MLLocation position.y: %f", position.y);


         //ROS_INFO("MLLocation newCost: %f", newCost); 

    }
    //ROS_INFO("MLLocation cost: %f", cost);
    //ROS_INFO("MLLocation after while"); 
    std::vector<double> distances = distanceToBeacons(position, rangingMeasurements);
    
    arma::mat jacobian(numMeasurements, 2);
    arma::vec realObservationError(numMeasurements);
 
    // Dado que existen casos de rangings con multitrayecto que se pueden dar por LOS, es conveniente
    // ser muy pesimista a la hora de devolver la covarianza.
    //
    // Se estima el MSE entre los rangings y la posicion estimada, y se supone que todos los rangings
    // están llegando con esa precisión. Así si un ranging llega con un desfase de metros, que descoloque
    // la posición estimada en gran proporción respecto a todas la balizas, se verá reflejado en grandes covarianzas
    // en todas ellas.
    double rangingError = estimationError(rangingMeasurements, position);
 
 //ROS_INFO("MLLocation after rangingError"); 
    // Covarianza estimada a partir de la linearizacion de la función de distancia.
    // Existe el peligro de que se subestime...
    for (int i = 0; i < rangingMeasurements.size(); ++i)
    {
        auto& r = rangingMeasurements[i];
        jacobian(i, 0) = (position.x - r.beacon.position.x)/distances[i];
        jacobian(i, 1) = (position.y - r.beacon.position.y)/distances[i];
 
        realObservationError(i) = std::max(r.errorEstimation, rangingError);
    }
 
    //arma::mat priorCovariance =arma::eye<arma::mat>(3,3);
    //arma::mat kalmanGain = priorCovariance*jacobian.t() *
    //                       inv_sympd(jacobian*priorCovariance*jacobian.t() + rangingError*arma::eye<arma::mat>(countValid,countValid));
    //arma::mat covarianceMatrix = (arma::eye<arma::mat>(3, 3) - kalmanGain*jacobian)*priorCovariance;
 
    //arma::mat covarianceMatrix = rangingError*inv(jacobian.t()*jacobian) ;
 
 //ROS_INFO("MLLocation before covarianceMatrix"); 
    arma::mat covarianceMatrix =inv(jacobian.t()*inv(arma::diagmat(realObservationError))*jacobian);
 
    //double pos11 = covarianceMatrix(1,1);
 
    position.covarianceMatrix = covarianceMatrix;
 
    return position;
}

/**
 * Estima la posición en base a la información de ranging y a las  posiciones
 * de los beacons.
 *
 * Utiliza una estimación de máxima verosimilitud mediante una búsqueda por
 * gradiente descendente. Se utiliza como función de verosimilitud la
 * discrepancia entre las distancias a las balizas de un punto dado respecto
 * a las medidas obtenidas.
 *
 * Debido a que se utiliza un algoritmo de gradiente, podrían darse casos en
 * los que el algoritmo diverge. Se puede minimizar esto pasando en el tercer
 * parámetro una estimación previa desde la que iniciar la búsqueda.
 */
Vector3 MLLocation::estimatePosition(const std::vector<RangingMeasurement>& rangingMeasurements,
                         const Vector3& previousEstimation) {
//    if (rangings.size() < 3) {
//        Vector3 position = { NAN, NAN, NAN };
//        return position;
//    }

    int numMeasurements = rangingMeasurements.size();
    if (numMeasurements < 3){
        Vector3 position;
        position.x = NAN;
        position.y = NAN;
        position.z = NAN;
        return position;
    }
    Vector3 position = previousEstimation;
    double cost = 1e20, newCost = 1;
    while((cost-newCost/cost > 1e-3) && (newCost> 1e-6)) {
        cost = newCost;
        std::vector<double> distances = distanceToBeacons(position, rangingMeasurements);
        arma::vec gradient(3, arma::fill::zeros);
        arma::mat secondDeriv(3, 3, arma::fill::zeros);
            for (int i = 0; i < rangingMeasurements.size(); ++i)
    {
        auto& r = rangingMeasurements[i];
             gradient(0) += (r.ranging - distances[i])*
                      (r.beacon.position.x - position.x)/(distances[i]*r.errorEstimation);
             gradient(1) += (r.ranging - distances[i])*
                      (r.beacon.position.y - position.y)/(distances[i]*r.errorEstimation);
             gradient(2) += (r.ranging - distances[i])*
                      (r.beacon.position.z - position.z)/(distances[i]*r.errorEstimation);

             double d3 = distances[i]*distances[i]*distances[i];
             secondDeriv(0, 0) += ( 1 - r.ranging/distances[i]
                                      + r.ranging*(r.beacon.position.x - position.x)*(r.beacon.position.x - position.x)/d3  
                                 )/r.errorEstimation;
             secondDeriv(1, 1) += ( 1 - r.ranging/distances[i]
                                      + r.ranging*(r.beacon.position.y - position.y)*(r.beacon.position.y - position.y)/d3  
                                 )/r.errorEstimation;
             secondDeriv(2, 2) += ( 1 - r.ranging/distances[i]
                                      + r.ranging*(r.beacon.position.z - position.z)*(r.beacon.position.z - position.z)/d3  
                                 )/r.errorEstimation;
             double diffXY = r.ranging*(r.beacon.position.x - position.x)*(r.beacon.position.y - position.y)/(d3*r.errorEstimation);
             double diffXZ = r.ranging*(r.beacon.position.x - position.x)*(r.beacon.position.z - position.z)/(d3*r.errorEstimation);
             double diffYZ = r.ranging*(r.beacon.position.y - position.y)*(r.beacon.position.z - position.z)/(d3*r.errorEstimation);
             secondDeriv(0, 1) += diffXY;
             secondDeriv(0, 2) += diffXZ;
             secondDeriv(1, 2) += diffYZ;
             secondDeriv(1, 0) += diffXY;
             secondDeriv(2, 0) += diffXZ;
             secondDeriv(2, 1) += diffYZ;
        }
        arma::vec pos(3);
        pos(0) = position.x;
        pos(1) = position.y;
        pos(2) = position.z;
        arma::vec rhs = secondDeriv*pos - gradient;
        arma::vec newPos = arma::solve(secondDeriv, rhs);
        position.x = newPos(0);
        position.y = newPos(1);
        position.z = newPos(2);
//        arma::vec shift = inv(secondDeriv)*gradient;
//        position.x = position.x - shift(0);
//        position.y = position.y - shift(1);
//        position.z = position.z - shift(2);
        distances = distanceToBeacons(position, rangingMeasurements);
        newCost = 0.0;
    for (int i = 0; i < rangingMeasurements.size(); ++i)
    {
        auto& r = rangingMeasurements[i];
            newCost += (r.ranging-distances[i])*(r.ranging-distances[i])/r.errorEstimation;
        }
        //ROS_INFO("MLLocation newCost: %f", newCost);
        //std::cout << newCost << std::endl;
    }

    std::vector<double> distances = distanceToBeacons(position, rangingMeasurements);
    arma::mat jacobian(numMeasurements, 3);
    arma::vec realObservationError(numMeasurements);

    // Dado que existen casos de rangings con multitrayecto que se pueden dar por LOS, es conveniente
    // ser muy pesimista a la hora de devolver la covarianza.
    //
    // Se estima el MSE entre los rangings y la posicion estimada, y se supone que todos los rangings
    // están llegando con esa precisión. Así si un ranging llega con un desfase de metros, que descoloque
    // la posición estimada en gran proporción respecto a todas la balizas, se verá reflejado en grandes covarianzas
    // en todas ellas.
    double rangingError = estimationError(rangingMeasurements, position);


    // Covarianza estimada a partir de la linearizacion de la función de distancia.
    // Existe el peligro de que se subestime...
    for (int i = 0; i < rangingMeasurements.size(); ++i)
    {
        auto& r = rangingMeasurements[i];
        jacobian(i, 0) = (position.x - r.beacon.position.x)/distances[i];
        jacobian(i, 1) = (position.y - r.beacon.position.y)/distances[i];
        jacobian(i, 2) = (position.z - r.beacon.position.z)/distances[i];

        realObservationError(i) = std::max(r.errorEstimation, rangingError);
    }

    //arma::mat priorCovariance =arma::eye<arma::mat>(3,3);
    //arma::mat kalmanGain = priorCovariance*jacobian.t() *
    //                       inv_sympd(jacobian*priorCovariance*jacobian.t() + rangingError*arma::eye<arma::mat>(countValid,countValid));
    //arma::mat covarianceMatrix = (arma::eye<arma::mat>(3, 3) - kalmanGain*jacobian)*priorCovariance;

    //arma::mat covarianceMatrix = rangingError*inv(jacobian.t()*jacobian) ;

    arma::mat covarianceMatrix =inv(jacobian.t()*inv(arma::diagmat(realObservationError))*jacobian);

    //double pos11 = covarianceMatrix(1,1);

    position.covarianceMatrix = covarianceMatrix;

    return position;
}

/*************
 * Estimacion usando los rangings de dos tags a la vez.
 * En relativePosT0 y relativePosT1 esta la posicion relativa de cada tag con respecto al centro del dron
 * Pueden llegar de 1 a 4 medidas de ranging de cada tag, si para una baliza el ranging es < 0 es que no hubo medida entre esa baliza y el tag
 * ****************/
Vector3 MLLocation::estimatePositionTwoTags( const std::vector<Beacon>& beacons, const std::vector<double>& rangingsT0,const std::vector<double>& errorEstimationsT0,
                        const std::vector<double>& rangingsT1,const std::vector<double>& errorEstimationsT1, const Vector3& previousEstimation) {
////    if (rangingsT0.size() < 3) {
////        Vector3 position = { NAN, NAN, NAN };
////        return position;
////    }
//
   Vector3 position = previousEstimation;
//    double step =5E-5;
//
//
//    Vector3 positionT0,positionT1;
//
//
//    for (int iter = 0; iter < 100; iter++) {
//        positionT0.x = position.x + relativePosTO.x;
//        positionT0.y = position.y + relativePosTO.y;
//        positionT0.z = position.z + relativePosTO.z;
//        positionT1.x = position.x + relativePosT1.x;
//        positionT1.y = position.y + relativePosT1.y;
//        positionT1.z = position.z + relativePosT1.z;
//
//        vector<double> distancesT0 = distanceToBeacons(positionT0, beacons);
//        vector<double> distancesT1 = distanceToBeacons(positionT1, beacons);
//        double gradientX = 0.0, gradientY = 0.0, gradientZ = 0.0;
//        for (int i = 0; i < (int) beacons.size(); i++) {
//            if (rangingsT0[i]>=0){
//            gradientX += (rangingsT0[i] - distancesT0[i])*
//                         (beacons[i].position.x - positionT0.x)/(distancesT0[i]*errorEstimationsT0[i]);
//            gradientY += (rangingsT0[i] - distancesT0[i])*
//                         (beacons[i].position.y - positionT0.y)/(distancesT0[i]*errorEstimationsT0[i]);
//            gradientZ += (rangingsT0[i] - distancesT0[i])*
//                         (beacons[i].position.z - positionT0.z)/(distancesT0[i]*errorEstimationsT0[i]);
//            }
//
//            if (rangingsT1[i]>=0){
//            gradientX += (rangingsT1[i] - distancesT1[i])*
//                         (beacons[i].position.x - positionT1.x)/(distancesT1[i]*errorEstimationsT1[i]);
//            gradientY += (rangingsT1[i] - distancesT1[i])*
//                         (beacons[i].position.y - positionT1.y)/(distancesT1[i]*errorEstimationsT1[i]);
//            gradientZ += (rangingsT1[i] - distancesT1[i])*
//                         (beacons[i].position.z - positionT1.z)/(distancesT1[i]*errorEstimationsT1[i]);
//            }
//        }
//        position.x -= step*gradientX;
//        position.y -= step*gradientY;
//        position.z -= step*gradientZ;
//    }
//
//    if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z)) {
//        position.x = 0;
//        position.y = 0;
//        position.z = 0;
//    }
//
//    positionT0.x = position.x + relativePosTO.x;
//    positionT0.y = position.y + relativePosTO.y;
//    positionT0.z = position.z + relativePosTO.z;
//    positionT1.x = position.x + relativePosT1.x;
//    positionT1.y = position.y + relativePosT1.y;
//    positionT1.z = position.z + relativePosT1.z;
//
//    vector<double> distancesT0 = distanceToBeacons(positionT0, beacons);
//    vector<double> distancesT1 = distanceToBeacons(positionT1, beacons);
//
//    int countValid=0;
//    for (auto r : rangingsT0) {
//        if (r >= 0) {
//            countValid++;
//        }
//    }
//    for (auto r : rangingsT1) {
//        if (r >= 0) {
//            countValid++;
//        }
//    }
//    arma::mat jacobian(countValid, 3);
//   // arma::vec realObservationError(countValid);
//
//    countValid = 0;
//    for (int i = 0; i < (int)beacons.size(); i++) {
//        if (rangingsT0[i] >= 0) {
//            jacobian(countValid, 0) = (positionT0.x -beacons[i].position.x )/(distancesT0[i]);
//            jacobian(countValid, 1) = (positionT0.y- beacons[i].position.y)/(distancesT0[i]);
//            jacobian(countValid, 2) = (positionT0.z - beacons[i].position.z)/(distancesT0[i]);
//            //realObservationError(countValid) = errorEstimationsT0[i];
//            countValid++;
//        }
//
//        if (rangingsT1[i] >= 0) {
//            jacobian(countValid, 0) = (positionT1.x -beacons[i].position.x )/(distancesT1[i]);
//            jacobian(countValid, 1) = (positionT1.y- beacons[i].position.y)/(distancesT1[i]);
//            jacobian(countValid, 2) = (positionT1.z - beacons[i].position.z)/(distancesT1[i]);
//            //realObservationError(countValid) = errorEstimationsT1[i];
//            countValid++;
//        }
//
//    }
//
//    double realEstimationError = estimationErrorTwoTags(beacons,rangingsT0, rangingsT1,position);
//
//    arma::mat priorCovariance =arma::eye<arma::mat>(3,3);
//    arma::mat kalmanGain = priorCovariance*jacobian.t() *
//                           inv(jacobian*priorCovariance*jacobian.t() + arma::eye<arma::mat>(countValid, countValid)*realEstimationError);
//    arma::mat covarianceMatrix = (arma::eye<arma::mat>(3, 3) - kalmanGain*jacobian)*priorCovariance;
//
   return position;
}

/**
 * Discrepancia entre las distancias de las balizas al punto position, y los
 * valores de ranging obtenidos desde las balizas.
 */
double MLLocation::estimationError(const std::vector<RangingMeasurement>& rangingMeasurements,
                       const Vector3& position) {
    if (rangingMeasurements.size() == 0) {
        return -1;
    }
    std::vector<double> distances = distanceToBeacons(position,rangingMeasurements);
    double error = 0.0;
    for (int i = 0; i < rangingMeasurements.size(); ++i)
    {
        auto& r = rangingMeasurements[i];
        error += (distances[i]-r.ranging)*
                 (distances[i]-r.ranging);
    }

    return error;
}


/**
 * Discrepancia entre las distancias de las balizas al punto position, y los
 * valores de ranging obtenidos desde las balizas.
 */
double MLLocation::estimationErrorTwoTags(const std::vector<Beacon>& beacons,
                                           const std::vector<double>& rangingsT0,
                                           const std::vector<double>& rangingsT1,
                       const Vector3& position) {
// 
//     Vector3 positionT0,positionT1;
// 
//     positionT0.x = position.x + relativePosTO.x;
//     positionT0.y = position.y + relativePosTO.y;
//     positionT0.z = position.z + relativePosTO.z;
//     positionT1.x = position.x + relativePosT1.x;
//     positionT1.y = position.y + relativePosT1.y;
//     positionT1.z = position.z + relativePosT1.z;
// 
//     vector<double> distancesT0 = distanceToBeacons(positionT0, beacons);
//     vector<double> distancesT1 = distanceToBeacons(positionT1, beacons);
// 
// 
//     double error = 0.0;
//     int count = 0;
//     for (int i = 0; i < (int) beacons.size(); i++) {
//         if (rangingsT0[i]>0) {
//             error += (distancesT0[i]-rangingsT0[i])*
//                      (distancesT0[i]-rangingsT0[i]);
//             count+=1;
//         }
//         if (rangingsT1[i]>0) {
//             error += (distancesT1[i]-rangingsT1[i])*
//                      (distancesT1[i]-rangingsT1[i]);
//             count+=1;
//         }
//     }
// 
//     if (count>0){
//         return error/count;
//     }
// 
//     //No se pudo estimar el error
    return -1;
}


void MLLocation::bestRangingsByDistance(const std::vector<Beacon>& beacons,
                                           const std::vector<double>& rangingsT0,
                                           const std::vector<double>& rangingsT1,
                       const Vector3& position, std::vector<RangingQuality>& rangingQualities) {

//     Vector3 positionT0,positionT1;
// 
//     positionT0.x = position.x + relativePosTO.x;
//     positionT0.y = position.y + relativePosTO.y;
//     positionT0.z = position.z + relativePosTO.z;
//     positionT1.x = position.x + relativePosT1.x;
//     positionT1.y = position.y + relativePosT1.y;
//     positionT1.z = position.z + relativePosT1.z;
// 
//     vector<double> distancesT0 = distanceToBeacons(positionT0, beacons);
//     vector<double> distancesT1 = distanceToBeacons(positionT1, beacons);
// 
//     int count = 0;
//     for (int i = 0; i < (int) beacons.size(); i++) {
// 
//         if (rangingsT0[i]>0) {
//             RangingQuality rangingQuality;
//             rangingQuality.tagId = 0;
//             rangingQuality.innerIndex = i;
//             rangingQuality.error = (distancesT0[i]-rangingsT0[i])*
//                     (distancesT0[i]-rangingsT0[i]);
// 
//             if (count==0){
//                 rangingQualities.push_back(rangingQuality);
//             } else {
//                 bool inserted = false;
//                 for (int j=0;j<count;j++){
//                     if (rangingQualities[j].error>rangingQuality.error){
//                         //Insertamos
//                         rangingQualities.insert(rangingQualities.begin() + j,rangingQuality);
//                         inserted = true;
//                         break;
//                     }
//                 }
//                 if (!inserted){
//                    rangingQualities.push_back(rangingQuality);
//                 }
//             }
//             count+=1;
//         }
//         if (rangingsT1[i]>0) {
//             RangingQuality rangingQuality;
//             rangingQuality.tagId = 1;
//             rangingQuality.innerIndex = i;
//             rangingQuality.error = (distancesT1[i]-rangingsT1[i])*(distancesT1[i]-rangingsT1[i]);
//             if (count==0){
//                 rangingQualities.push_back(rangingQuality);
//             } else {
//                 bool inserted = false;
//                 for (int j=0;j<count;j++){
//                     if (rangingQualities[j].error>rangingQuality.error){
//                         //Insertamos
//                         rangingQualities.insert(rangingQualities.begin() + j,rangingQuality);
//                         inserted = true;
//                         break;
//                     }
//                 }
//                 if (!inserted){
//                    rangingQualities.push_back(rangingQuality);
//                 }
//             }
//             count+=1;
//         }
//     }
}

void MLLocation::bestRangingsByDistance(const std::vector<RangingMeasurement>& rangingMeasurements,
                       const Vector3& position, std::vector<RangingQuality>& rangingQualities) {

    std::vector<double> distancesT0 = distanceToBeacons(position, rangingMeasurements);

    for (int i = 0; i < rangingMeasurements.size(); ++i)
    {
        auto& r = rangingMeasurements[i];
         RangingQuality rangingQuality;
         rangingQuality.tagId = 0;
         rangingQuality.innerIndex = i;
         rangingQuality.error = (distancesT0[i]-r.ranging)*
                 (distancesT0[i]-r.ranging);
         rangingQualities.push_back(rangingQuality);
    }
    std::sort(rangingQualities.begin(), rangingQualities.end(), [](const RangingQuality& a, const RangingQuality& b) { return a.error < b.error; });
}

Vector3 MLLocation::estimatePositionTwoTagsIgnoreN( const std::vector<Beacon>& beacons, const std::vector<double>& rangingsT0,const std::vector<double>& errorEstimationsT0,
                       const std::vector<double>& rangingsT1,const std::vector<double>& errorEstimationsT1, const Vector3& previousEstimation, int numRangingsToIgnore) {

//     Vector3 position;
//     position= estimatePositionTwoTags(beacons, rangingsT0,errorEstimationsT0,rangingsT1,errorEstimationsT1,previousEstimation);
// 
//     vector<RangingQuality> rangingQualities;
//     bestRangingsByDistance(beacons,rangingsT0,rangingsT1, position, rangingQualities);
// 
//     vector<double> newRangingsT0(rangingsT0);
//     vector<double> newRangingsT1(rangingsT1);
// 
//     int maxRangings = rangingsT0.size() + rangingsT1.size();
//     for (int i=(maxRangings-1);i>=(maxRangings-numRangingsToIgnore);i--){
//         RangingQuality rangingQuality = rangingQualities[i];
//         if (rangingQuality.tagId==0){
//             newRangingsT0[rangingQuality.innerIndex] = -1.0;
//         } else {
//             newRangingsT1[rangingQuality.innerIndex] = -1.0;
//         }
//     }

    return estimatePositionTwoTags(beacons, rangingsT0,errorEstimationsT0,rangingsT1,errorEstimationsT1,previousEstimation);
}


Vector3 MLLocation::estimatePositionTwoTagsBestGroup( const std::vector<Beacon>& beacons, const std::vector<double>& rangingsT0,const std::vector<double>& errorEstimationsT0,
                       const std::vector<double>& rangingsT1,const std::vector<double>& errorEstimationsT1, const Vector3& previousEstimation, int bestCriteria, double minZ, double maxZ) {

    Vector3 position= estimatePositionTwoTags(beacons, rangingsT0,errorEstimationsT0,rangingsT1,errorEstimationsT1,previousEstimation);

    std::vector<Vector3> groupPositions;
// 
//     //Position 0,1,2
//     vector<double> newRangingsT0C0(rangingsT0);
//     vector<double> newRangingsT1C0(rangingsT1);
//     newRangingsT0C0[3] = -1.0;
//     newRangingsT1C0[3] = -1.0;
//     Vector3 positionC0= estimatePositionTwoTags(beacons, newRangingsT0C0,errorEstimationsT0,newRangingsT1C0,errorEstimationsT1,previousEstimation);
//     groupPositions.push_back(positionC0);
//     //Position 0,1,3
//     vector<double> newRangingsT0C1(rangingsT0);
//     vector<double> newRangingsT1C1(rangingsT1);
//     newRangingsT0C1[2] = -1.0;
//     newRangingsT1C1[2] = -1.0;
//     Vector3 positionC1= estimatePositionTwoTags(beacons, newRangingsT0C1,errorEstimationsT0,newRangingsT1C1,errorEstimationsT1,previousEstimation);
//     groupPositions.push_back(positionC1);
//     //Position 0,2,3
//     vector<double> newRangingsT0C2(rangingsT0);
//     vector<double> newRangingsT1C2(rangingsT1);
//     newRangingsT0C2[1] = -1.0;
//     newRangingsT1C2[1] = -1.0;
//     Vector3 positionC2= estimatePositionTwoTags(beacons, newRangingsT0C2,errorEstimationsT0,newRangingsT1C2,errorEstimationsT1,previousEstimation);
//     groupPositions.push_back(positionC2);
//     //Position 1,2,3
//     vector<double> newRangingsT0C3(rangingsT0);
//     vector<double> newRangingsT1C3(rangingsT1);
//     newRangingsT0C3[0] = -1.0;
//     newRangingsT1C3[0] = -1.0;
//     Vector3 positionC3= estimatePositionTwoTags(beacons, newRangingsT0C3,errorEstimationsT0,newRangingsT1C3,errorEstimationsT1,previousEstimation);
//     groupPositions.push_back(positionC3);
// 
// 
//     double minError = 0;
//     int minIndex = -1;
//     double currentError = 0.0;
// 
//     for (int i=0;i<4;i++){
//         if ((groupPositions[i].z>=minZ) && (groupPositions[i].z<=maxZ)){
//             if (bestCriteria==SELECT_ANCHORS_USING_ERROR_XYZ) {
//                 //Using X,Y,Z errors
//                 currentError = groupPositions[i].covarianceMatrix(0,0) + groupPositions[i].covarianceMatrix(1,1) + groupPositions[i].covarianceMatrix(2,2);
//             } else {
//                 //Only Z
//                 currentError = groupPositions[i].covarianceMatrix(2,2);
//             }
//             if (minIndex==-1){
//                 minIndex = i;
//                 minError = currentError;
//             }
// 
//             if (currentError<=minError){
//                 minIndex = i;
//                 minError = currentError;
//             }
//         }
//     }

    int minIndex = -1;
    if (minIndex==-1){
        //Ninguna de los subgrupos da una estimacion con Z en los margenes permitidos
        return position;
    } else {
        return groupPositions[minIndex];
    }
}



Vector3 MLLocation::estimatePositionIgnoreN( const std::vector<RangingMeasurement>& rangingMeasurements,
                       const Vector3& previousEstimation, int numRangingsToIgnore) {

    Vector3 position= estimatePosition(rangingMeasurements, previousEstimation);

    std::vector<RangingQuality> rangingQualities;
    bestRangingsByDistance(rangingMeasurements, previousEstimation, rangingQualities);

    std::vector<RangingMeasurement> newRangings(rangingMeasurements);

    int i=0;
    for (const auto& q : rangingQualities) {
        newRangings[i] = rangingMeasurements[q.innerIndex];
        i++;
    }

    for (int i = 0; i < numRangingsToIgnore; i++) {
        newRangings.pop_back();
    }

    return estimatePosition(newRangings,previousEstimation);
}


Vector3 MLLocation::estimatePositionIgnoreN( const std::vector<RangingMeasurement>& rangingMeasurements,
                       const Vector3& previousEstimation, int numRangingsToIgnore, std::vector<RangingMeasurement>& newRangingMeasurements) {

    Vector3 position= estimatePosition2D(rangingMeasurements, previousEstimation);

    std::vector<RangingQuality> rangingQualities;
    bestRangingsByDistance(rangingMeasurements, previousEstimation, rangingQualities);

    //std::vector<RangingMeasurement> newRangings(rangingMeasurements);


    int i=0;

    while (i<rangingMeasurements.size() - numRangingsToIgnore){
        const auto& q = rangingQualities[i];
        newRangingMeasurements.push_back(rangingMeasurements[q.innerIndex]);
        i++;
    }

    return estimatePosition2D(newRangingMeasurements,previousEstimation);
}


Vector3 MLLocation::estimatePositionBestGroup(const std::vector<RangingMeasurement>& rangingMeasurements,
                       const Vector3& previousEstimation, int bestCriteria, double minZ, double maxZ) {

    Vector3 position = estimatePosition(rangingMeasurements, previousEstimation);

    int numMeasurements = rangingMeasurements.size();

    if (numMeasurements<4){
        //Si no tenemos al menos 4, no podemos quitar ninguno porque nos quedariamos con menos de 3
        return position;
    }

    std::vector<Vector3> groupPositions;

    //Position 0,1,2
    std::vector<RangingMeasurement> newRangingsC0(rangingMeasurements);
    newRangingsC0.erase(newRangingsC0.begin() + 3);
    Vector3 positionC0= estimatePosition(newRangingsC0,previousEstimation);
    groupPositions.push_back(positionC0);
    //Position 0,1,3
    std::vector<RangingMeasurement> newRangingsC1(rangingMeasurements);
    newRangingsC0.erase(newRangingsC1.begin() + 2);
    Vector3 positionC1= estimatePosition(newRangingsC1,previousEstimation);
    groupPositions.push_back(positionC1);
    //Position 0,2,3
    std::vector<RangingMeasurement> newRangingsC2(rangingMeasurements);
    newRangingsC0.erase(newRangingsC2.begin() + 1);
    Vector3 positionC2= estimatePosition(newRangingsC2,previousEstimation);
    groupPositions.push_back(positionC2);
    //Position 1,2,3
    std::vector<RangingMeasurement> newRangingsC3(rangingMeasurements);
    newRangingsC0.erase(newRangingsC3.begin());
    Vector3 positionC3= estimatePosition(newRangingsC3,previousEstimation);
    groupPositions.push_back(positionC3);


    double minError = 0;
    int minIndex = -1;
    double currentError = 0.0;

    for (int i=0;i<4;i++){
        if ((groupPositions[i].z>=minZ) && (groupPositions[i].z<=maxZ)){
            if (bestCriteria==SELECT_ANCHORS_USING_ERROR_XYZ) {
                //Using X,Y,Z errors
                currentError = groupPositions[i].covarianceMatrix(0,0) + groupPositions[i].covarianceMatrix(1,1) + groupPositions[i].covarianceMatrix(2,2);
            } else {
                //Only Z
                currentError = groupPositions[i].covarianceMatrix(2,2);
            }
            if (minIndex==-1){
                minIndex = i;
                minError = currentError;
            }

            if (currentError<=minError){
                minIndex = i;
                minError = currentError;
            }
        }
    }

    if (minIndex==-1){
        //Ninguna de los subgrupos da una estimacion con Z en los margenes permitidos
        return position;
    } else {
        return groupPositions[minIndex];
    }
}
