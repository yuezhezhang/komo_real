void RRT_SingleTree::projectToManifold(arr& q) {
  rai::Frame* f0 = DISP.getFrame("l_gripper");
  rai::Frame* f1 = DISP.getFrame("r_gripper");
  arr target_positionDiff = {-0.43,0,0}; //
  arr target_positionRel = {0,0,-0.43}; 
  arr target_poseRel = {0,0,-0.43,0,1,0,0}; 
  arr target_poseDiff = {-0.43, 0., 0.0, 0, 1, 1, 0};
  arr x = {1, 1, 1};
  arr pos0, pos1, delta_q, J, J_pseudo;
  arr qua;
  arr y1, J1, y2, J2;
  auto constraint= FS_poseDiff; //add 
  int iter=1;
  // TODO need to consider constraints and collision together 
  while (length(x) >= 1e-6 && iter < 15) {
    DISP.setJointState(q);
    // DISP.view(true);

    pos0 = DISP.kinematics_pos(f0); 
    // arr rot0 = DISP.kinematicsQuat(f0);
    pos1 = DISP.kinematics_pos(f1);
    
  
    // DISP.kinematicsQuat(y1,J1,f0);

    // // DISP.kinematicsQuat(y2,J2,f1);
    // if(scalarProduct(y1, y2)>=0.) {
    //   y1 -= y2;
    //   J1 -= J2;
    // } else {
    //   y1 += y2;
    //   J1 += J2;
    // }

    if (constraint == FS_positionDiff) {
      // cout<<"pos0 -pos1: "<<pos0 -pos1<<endl;
      x = pos0 -pos1 -target_positionDiff;
      // x = pos0 -pos1;
      J = *pos0.jac;
    } else if (constraint == FS_positionRel) {
      arr a1_rot_matrix = f1->ensure_X().rot.getArr();
      // x = ~a1_rot_matrix * (pos0 - pos1) - constraint->target;
      x = pos0 - pos1 - a1_rot_matrix * target_positionRel;
      // x = pos0 - pos1;
      J = *pos0.jac; //  - *pos1.jac; // below works only for 2 robots
      // arr A;
      // DISP.jacobian_angular(A, f1); // A: [3, 16] left [3, 8] is zero
      // // cout << "A " << A << endl; 
      // // cout << "A (y1-y2) " << crossProduct(A, pos0 - pos1) << endl; // only works for [3, n]*3
      // J -= crossProduct(A, pos0 - pos1);
    } else if (constraint == FS_poseRel) {

        qua = F_PoseRel().eval({f0,f1});
        // cout<<"qua: "<<qua<< endl;
        x = qua - target_poseRel;
        J = *qua.jac;
    } else if (constraint == FS_poseDiff) {
        // cout<< F_PoseDiff().eval({f0,f1});

        qua = F_PoseDiff().eval({f0,f1});
        // cout<<"qua: "<<qua<< endl;
        x = qua - target_poseDiff;
        J = *qua.jac;
    }

    J_pseudo = pseudoInverse(J);

    // Newton Method
    delta_q = J_pseudo * x;
    q -= delta_q;
    iter += 1;
  }

  // if (length(pos0-focus1) + length(pos0-focus2)< e*length(focus1-focus2) && 
  //     length(pos1-focus1) + length(pos1-focus2)< e*length(focus1-focus2)) {
  //     within_ellipse = true;
  // } else {
  //     within_ellipse = false;
  //     // DISP.view(true);
  // }
  // if (length(x) > 1e-6) {
  //   cout << "projection not working: len " << length(x) << endl;
  //   within_ellipse = false;
  // }
}