while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
			{


				std::cout<<"Thread ID: "<<thread_id<<std::endl;
				tcurr = sutil::CSystemClock::getSysTime();

				Eigen::Vector3d end_eff_pos=rtask_hand->x_;



				double error=0.0;


				int sec_iter=35000;
				//if(iter<20000 && inter_goal==false)
				//{
				//    SharedPosition<<0,0,1;
				//    R_des1<<1,0,0,0,0,1,0,-1,0;
				//}

				Eigen::VectorXd Goal(3);
				if(inter1_goal==false)
				{
					Goal=pos;

					if(loop_ctr % update_period == 0) {
						//rtask_hand->x_goal_(0) = 0.15*sin(tcurr-tstart)-0.15;
						//rtask_hand->x_goal_(1) = 0.25*cos(tcurr-tstart);
						//rtask_hand->x_goal_(2) = -0.25;

						// IMPORTANT: set target values
						IP->TargetPositionVector->VecData[0] = Goal(0);
						IP->TargetPositionVector->VecData[1] = Goal(1);
						IP->TargetPositionVector->VecData[2] = Goal(2);

						IP->TargetVelocityVector->VecData[0] = 0.0;
						IP->TargetVelocityVector->VecData[1] = 0.0;
						IP->TargetVelocityVector->VecData[2] = 0.0;
					}
					loop_ctr++;


					// ResultValue contains status information
					int ResultValue	= RML->RMLPosition(*IP, OP, Flags);


					// IMPORTANT: set the data to some initial values for the current state
					*IP->CurrentPositionVector	=	*OP->NewPositionVector		;
					*IP->CurrentVelocityVector	=	*OP->NewVelocityVector		;
					*IP->CurrentAccelerationVector	=	*OP->NewAccelerationVector	;

					for(int i = 0; i < dof; i++) SharedPosition(i) = OP->NewPositionVector->VecData[i];
					R_des1<<1,0,0,0,0,1,0,-1,0;

//					rtask_wrist->has_been_activated_=true;
					//              rtask_gc->has_been_activated_=false;

					error=(Goal-rtask_hand->x_).norm();
					std::cout <<"\nnorm error" <<error;
					if (error<0.02){
						inter1_goal=true;
						cur_iter=iter;}

				}
				else if(inter_goal==false)
				{

					/* ---Traj gen code---*/


					Goal(0)=tray_pos(0);
					Goal(1)=tray_pos(1)-tray_length-0.3;
					Goal(2)=tray_pos(2);
					if(loop_ctr % update_period == 0) {
						//rtask_hand->x_goal_(0) = 0.15*sin(tcurr-tstart)-0.15;
						//rtask_hand->x_goal_(1) = 0.25*cos(tcurr-tstart);
						//rtask_hand->x_goal_(2) = -0.25;

						// IMPORTANT: set target values
						IP->TargetPositionVector->VecData[0] = Goal(0);
						IP->TargetPositionVector->VecData[1] = Goal(1);
						IP->TargetPositionVector->VecData[2] = Goal(2);

						IP->TargetVelocityVector->VecData[0] = 0.0;
						IP->TargetVelocityVector->VecData[1] = 0.0;
						IP->TargetVelocityVector->VecData[2] = 0.0;
					}
					loop_ctr++;

					// IMPORTANT: set the data to some initial values for the current state
					*IP->CurrentPositionVector	=	*OP->NewPositionVector		;
					*IP->CurrentVelocityVector	=	*OP->NewVelocityVector		;
					*IP->CurrentAccelerationVector	=	*OP->NewAccelerationVector	;

					// ResultValue contains status information
					int ResultValue	= RML->RMLPosition(*IP, OP, Flags);
					for(int i = 0; i < dof; i++) SharedPosition(i) = OP->NewPositionVector->VecData[i];
					R_des1<<1,0,0,0,0,1,0,-1,0;
//					rtask_wrist->has_been_activated_=true;
					//              rtask_gc->has_been_activated_=false;

					error=(Goal-rtask_hand->x_).norm();
					std::cout <<"\nnorm error" <<error;
					if (error<0.02){
						inter_goal=true;
						cur_iter=iter;}
				}
				/*else if(iter<sec_iter+20000)
                        {
                            SharedPosition<<tray_pos(0),tray_pos(1)-tray_length-0.3,tray_pos(2);
                            R_des1<<1,0,0,0,0,1,0,-1,0;
                        }*/

				//          else if(inter_goal==true && iter<cur_iter+20000)
				//          { std::cout <<"\n REACHED THIS LOOP";
				//              if(iter%100==0)
				//              {
				//                  if(SharedPosition(1)<tray_pos(1) - 0.3)
				//                      SharedPosition(1)+=0.005;
				//                  R_des1<<1,0,0,0,0,1,0,-1,0;
				//              }
				//          }
				//          else if(inter_goal==true && iter<cur_iter+30000)
				//          {
				//              movetray=false;
				//              if(iter%100==0)
				//              {
				//                  SharedPosition(1)-=0.005;
				//                  R_des1<<1,0,0,0,0,1,0,-1,0;
				//              }

				//          }


				rtask_hand->x_goal_ =SharedPosition;
//				rtask_wrist->lam_goal_=R_des1;

				//             rtask_gc->q_goal_<<0,0,0,0,0,0,0;
				//             rtask_gc->dq_goal_<<0,0,0,0,0,0,0;
				//             rtask_gc->ddq_goal_<<0,0,0,0,0,0,0;


//				std::cout<<"\nDelta Phi: "<<rtask_wrist->delta_phi_<<"\nLambda Goal: "<<rtask_wrist->lam_goal_.coeffs().transpose();
//				std::cout<<"\nLambda Current: "<<rtask_wrist->lam_.coeffs().transpose();


				std::cout<<"\nForce pos: "<<rtask_hand->force_task_.transpose() <<"\ndeltax:"<< rtask_hand->x_goal_-rtask_hand->x_.transpose();
				std::cout<<"\nQ:" <<rio.sensors_.q_.transpose();


				// Compute control forces (note that these directly have access to the io data ds).
				rctr.computeDynamics();
				rctr.computeControlForces();

				// DEBUG
//					rio.actuators_.force_gc_commanded_.setZero(7);

				// Integrate the dynamics
				//dyn_tao.integrate(rio,dt);

				iter++;

				/**************************************SET POSTIION OF GRAPHICS OBJECTS***************************/

				//SET PLANE POSITION
				Eigen::MatrixXd T_matrix;
				scl::SRigidBodyDyn *rbd=rgcm.rbdyn_tree_.at("link7");
				T_matrix=rbd->T_o_lnk_.matrix();
				Eigen::Vector4d zero_vector(0,0,0,1);
				Eigen::Vector4d plane_pos=T_matrix*zero_vector;
				chai3d::cMatrix3d rot_matrix ,_90matrix;
				rot_matrix.set(T_matrix(0,0),T_matrix(0,1),T_matrix(0,2),T_matrix(1,0),T_matrix(1,1),T_matrix(1,2),T_matrix(2,0),T_matrix(2,1),T_matrix(2,2));
				_90matrix.set(1,0,0,0,0,1,0,-1,0);
				rot_matrix=rot_matrix*_90matrix;
				rchai.getChaiData()->meshes_rendered_.at("plane")->graphics_obj_->setLocalRot(rot_matrix);
				if(movetray)
					rchai.getChaiData()->meshes_rendered_.at("plane")->graphics_obj_->setLocalPos(plane_pos[0],plane_pos[1],plane_pos[2]);

				//SET GOAL SPHERE POSITION
				goal_sphere->setLocalPos(rtask_hand->x_goal_[0],rtask_hand->x_goal_[1],rtask_hand->x_goal_[2]);

				/**************************************SET POSTIION OF GRAPHICS OBJECTS***************************/



				/* if(iter % 5000 == 0){std::cout<<"\nLam-goal"<<rtask_wrist->lam_goal_.coeffs().transpose()
          <<"\nlam-actual"<<rtask_wrist->lam_.coeffs().transpose()
          <<"\ndphi"<<rtask_wrist->delta_phi_.transpose();

        }*/
			}
