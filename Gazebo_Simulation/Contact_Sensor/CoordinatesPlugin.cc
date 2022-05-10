#include "CoordinatesPlugin.hh"


using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(CoordinatesPlugin)

CoordinatesPlugin::CoordinatesPlugin() : ModelPlugin ()
{
}
CoordinatesPlugin::~CoordinatesPlugin()
{
}
//Method for matrix multiplication
std::vector<float> CoordinatesPlugin::homotrans(geometry_msgs::TransformStamped transformStamped,geometry_msgs::Vector3Stamped v3Stamped)
{
    std::vector<float> homogeneous_vector = this->homo_v(v3Stamped);
    /*
    std::cout << "Homogeneous vector: " << homogeneous_vector[0] << " " << homogeneous_vector[1]
              << " " << homogeneous_vector[2] <<" " << homogeneous_vector[3] << std::endl;
    */
    std::vector<std::vector<float>> homogeneous_matrix = this->homo_matrix(transformStamped);
    /*
    std::cout << "Homogeneous matrix:\n"
              << homogeneous_matrix[0][0] << " " << homogeneous_matrix[0][1]
              << " " << homogeneous_matrix[0][2] << " " << homogeneous_matrix[0][3] << "\n"
              << homogeneous_matrix[1][0] << " " << homogeneous_matrix[1][1]<< " "
              <<homogeneous_matrix[1][2] << " " << homogeneous_matrix[1][3] << "\n"
              << homogeneous_matrix[2][0] << " " << homogeneous_matrix[2][1]<< " "
              <<homogeneous_matrix[2][2] << " " << homogeneous_matrix[2][3] << "\n"
              << homogeneous_matrix[3][0] << " " << homogeneous_matrix[3][1]<< " "
              <<homogeneous_matrix[3][2] << " " << homogeneous_matrix[3][3] << "\n"<<std::endl;
    */
    std::vector<float> result;
    result.resize(3);
    for(int row=0; row<homogeneous_matrix.size()-1;row++)
    {
        for(int column=0; column<homogeneous_matrix[0].size();column++)
        {
            result[row] = result[row] + homogeneous_matrix[row][column]*homogeneous_vector[column];
        }
    }
    //std::cout<<"Result: "<< result[0] << result[1] << result[2] << std::endl;
    return result;
}
//Method for extracting the homogeneous matrix from the tf2 transformation
std::vector<std::vector<float>> CoordinatesPlugin::homo_matrix(geometry_msgs::TransformStamped transformStamped)
{
    float q0,q1,q2,q3,dx,dy,dz;
    std::vector<std::vector<float>> homo_matrix;
    //Matrix resizing
    homo_matrix.resize(4,std::vector<float>(4,0));
    q0 = transformStamped.transform.rotation.x;
    q1 = transformStamped.transform.rotation.y;
    q2 = transformStamped.transform.rotation.z;
    q3 = transformStamped.transform.rotation.w;
    dx = transformStamped.transform.translation.x;
    dy = transformStamped.transform.translation.y;
    dz = transformStamped.transform.translation.z;
    homo_matrix[0][0] = 2*(pow(q0,2)+pow(q1,2))-1;
    homo_matrix[0][1] = 2*(q1*q2-q0*q3);
    homo_matrix[0][2] = 2*(q1*q3+q0*q2);
    homo_matrix[0][3] = dx;
    homo_matrix[1][0] = 2*(q1*q2+q0*q3);
    homo_matrix[1][1] = 2*(pow(q0,2)+pow(q2,2))-1;
    homo_matrix[1][2] = 2*(q2*q3-q0*q1);
    homo_matrix[1][3] = dy;
    homo_matrix[2][0] = 2*(q1*q3-q0*q2);
    homo_matrix[2][1] = 2*(q2*q3+q0*q1);
    homo_matrix[2][2] = 2*(pow(q0,2)+pow(q3,2))-1;
    homo_matrix[2][3] = dz;
    homo_matrix[3][0] = 0;
    homo_matrix[3][1] = 0;
    homo_matrix[3][2] = 0;
    homo_matrix[3][3] = 1;

    return homo_matrix;
}
//Method for converting a 3D vector into its homogeneous representation
std::vector<float> CoordinatesPlugin::homo_v(geometry_msgs::Vector3Stamped v3Stamped)
{
  std::vector<float> result;
    result.push_back(v3Stamped.vector.x);
    result.push_back(v3Stamped.vector.y);
    result.push_back(v3Stamped.vector.z);
    result.push_back(1.0);

    return result;
}
//Method to get the name of the sensor from the correspondent collission name

std::string CoordinatesPlugin::getName(std::string str)
{
    std::string delim = "::";
    std::string appendix = "_bracket";
    size_t location1 = str.find_last_of(delim);
    size_t location2 = str.find(appendix);
    location1 = location1+1;
    return str.substr(location1,location2-location1);
}

//Method to calculate the Eucledian distance between two points in the 3D space
float CoordinatesPlugin::dist(std::vector<float> P1,std::vector<float> P2)
{
    float distance_sqrd = 0;
    int size1 = P1.size();
    int size2 = P2.size();;
    if (size1 != size2)
    {
        try
        {
          throw 20;
        }
        catch (int exception)
        {
          std::cout << "An exception occurred. Incompatible sizes for distance calculation" << std::endl;
        }
    }

        distance_sqrd = (P1[0]-P2[0])*(P1[0]-P2[0])+(P1[1]-P2[1])*(P1[1]-P2[1])
                        +(P1[2]-P2[2])*(P1[2]-P2[2]);

    return pow(distance_sqrd,0.5);
}

//Method to build the tree of taxels locations
void CoordinatesPlugin::taxelsTreeBld()
{
    // Vectors resizing
    this->boh_taxels.resize(109); //This size needs to be changed later on!!!!!!!!!!!!!!!!!!!!!!!!!!
    this->palm_taxels.resize(121);
    this->ipb_taxels.resize(78);
    this->mpb_taxels.resize(78);
    this->ppb_taxels.resize(78);
    this->ipf_taxels.resize(65);
    this->mpf_taxels.resize(65);
    this->ppf_taxels.resize(65);
    this->tmb_taxels.resize(47);
    this->imb_taxels.resize(30);
    this->mmb_taxels.resize(30);
    this->pmb_taxels.resize(30);
    this->tmf_taxels.resize(31);
    this->imf_taxels.resize(27);
    this->mmf_taxels.resize(27);
    this->pmf_taxels.resize(27);
    this->tft_taxels.resize(66);
    this->ift_taxels.resize(66);
    this->mft_taxels.resize(66);
    this->pft_taxels.resize(66);
    // Vectors resizing
    this->trans_boh_taxels.resize(109,std::vector<float>(3,0));
    this->trans_palm_taxels.resize(121,std::vector<float>(3,0));
    this->trans_ipb_taxels.resize(78,std::vector<float>(3,0));
    this->trans_mpb_taxels.resize(78,std::vector<float>(3,0));
    this->trans_ppb_taxels.resize(78,std::vector<float>(3,0));
    this->trans_ipf_taxels.resize(65,std::vector<float>(3,0));
    this->trans_mpf_taxels.resize(65,std::vector<float>(3,0));
    this->trans_ppf_taxels.resize(65,std::vector<float>(3,0));
    this->trans_tmb_taxels.resize(47,std::vector<float>(3,0));
    this->trans_imb_taxels.resize(30,std::vector<float>(3,0));
    this->trans_mmb_taxels.resize(30,std::vector<float>(3,0));
    this->trans_pmb_taxels.resize(30,std::vector<float>(3,0));
    this->trans_tmf_taxels.resize(31,std::vector<float>(3,0));
    this->trans_imf_taxels.resize(27,std::vector<float>(3,0));
    this->trans_mmf_taxels.resize(27,std::vector<float>(3,0));
    this->trans_pmf_taxels.resize(27,std::vector<float>(3,0));
    this->trans_tft_taxels.resize(66,std::vector<float>(3,0));
    this->trans_ift_taxels.resize(66,std::vector<float>(3,0));
    this->trans_mft_taxels.resize(66,std::vector<float>(3,0));
    this->trans_pft_taxels.resize(66,std::vector<float>(3,0));
    // Specifying the taxels positions w.r.t. to the sensors centroids (P.S. Reconsider using push_back instead)
    //*******************************************************************
    //P.S. These coordinates are just approximations for now to debug the algorithm. They need to be adjusted later on!!!!
    //****************************************************************
    //BOH
    //Row1
    this->boh_taxels[0].vector.x = 0.047;
    this->boh_taxels[0].vector.y = -0.019;
    this->boh_taxels[0].vector.z = -0.009;
    this->boh_taxels[0].header.frame_id = "boh_bracket";
    this->boh_taxels[1].vector.x = 0.038;
    this->boh_taxels[1].vector.y = -0.019;
    this->boh_taxels[1].vector.z = -0.009;
    this->boh_taxels[1].header.frame_id = "boh_bracket";
    this->boh_taxels[2].vector.x = 0.03;
    this->boh_taxels[2].vector.y = -0.019;
    this->boh_taxels[2].vector.z = -0.009;
    this->boh_taxels[2].header.frame_id = "boh_bracket";
    this->boh_taxels[3].vector.x = 0.021;
    this->boh_taxels[3].vector.y = -0.019;
    this->boh_taxels[3].vector.z = -0.009;
    this->boh_taxels[3].header.frame_id = "boh_bracket";
    this->boh_taxels[4].vector.x = 0.013;
    this->boh_taxels[4].vector.y = -0.019;
    this->boh_taxels[4].vector.z = -0.009;
    this->boh_taxels[4].header.frame_id = "boh_bracket";
    this->boh_taxels[5].vector.x = 0.004;
    this->boh_taxels[5].vector.y = -0.019;
    this->boh_taxels[5].vector.z = -0.009;
    this->boh_taxels[5].header.frame_id = "boh_bracket";
    this->boh_taxels[6].vector.x = -0.005;
    this->boh_taxels[6].vector.y = -0.019;
    this->boh_taxels[6].vector.z = -0.009;
    this->boh_taxels[6].header.frame_id = "boh_bracket";
    this->boh_taxels[7].vector.x = -0.014;
    this->boh_taxels[7].vector.y = -0.019;
    this->boh_taxels[7].vector.z = -0.009;
    this->boh_taxels[7].header.frame_id = "boh_bracket";
    this->boh_taxels[8].vector.x = -0.023;
    this->boh_taxels[8].vector.y = -0.019;
    this->boh_taxels[8].vector.z = -0.009;
    this->boh_taxels[8].header.frame_id = "boh_bracket";
    this->boh_taxels[9].vector.x = -0.032;
    this->boh_taxels[9].vector.y = -0.019;
    this->boh_taxels[9].vector.z = -0.009;
    this->boh_taxels[9].header.frame_id = "boh_bracket";
    this->boh_taxels[10].vector.x = -0.041;
    this->boh_taxels[10].vector.y = -0.019;
    this->boh_taxels[10].vector.z = -0.009;
    this->boh_taxels[10].header.frame_id = "boh_bracket";
    this->boh_taxels[11].vector.x = -0.05;
    this->boh_taxels[11].vector.y = -0.019;
    this->boh_taxels[11].vector.z = -0.009;
    this->boh_taxels[11].header.frame_id = "boh_bracket";
    //Row2
    this->boh_taxels[12].vector.x = 0.047;
    this->boh_taxels[12].vector.y = -0.019;
    this->boh_taxels[12].vector.z = -0.016;
    this->boh_taxels[12].header.frame_id = "boh_bracket";
    this->boh_taxels[13].vector.x = 0.038;
    this->boh_taxels[13].vector.y = -0.019;
    this->boh_taxels[13].vector.z = -0.016;
    this->boh_taxels[13].header.frame_id = "boh_bracket";
    this->boh_taxels[14].vector.x = 0.03;
    this->boh_taxels[14].vector.y = -0.019;
    this->boh_taxels[14].vector.z = -0.016;
    this->boh_taxels[14].header.frame_id = "boh_bracket";
    this->boh_taxels[15].vector.x = 0.021;
    this->boh_taxels[15].vector.y = -0.019;
    this->boh_taxels[15].vector.z = -0.016;
    this->boh_taxels[15].header.frame_id = "boh_bracket";
    this->boh_taxels[16].vector.x = 0.013;
    this->boh_taxels[16].vector.y = -0.019;
    this->boh_taxels[16].vector.z = -0.016;
    this->boh_taxels[16].header.frame_id = "boh_bracket";
    this->boh_taxels[17].vector.x = 0.004;
    this->boh_taxels[17].vector.y = -0.019;
    this->boh_taxels[17].vector.z = -0.016;
    this->boh_taxels[17].header.frame_id = "boh_bracket";
    this->boh_taxels[18].vector.x = -0.005;
    this->boh_taxels[18].vector.y = -0.019;
    this->boh_taxels[18].vector.z = -0.016;
    this->boh_taxels[18].header.frame_id = "boh_bracket";
    this->boh_taxels[19].vector.x = -0.014;
    this->boh_taxels[19].vector.y = -0.019;
    this->boh_taxels[19].vector.z = -0.016;
    this->boh_taxels[19].header.frame_id = "boh_bracket";
    this->boh_taxels[20].vector.x = -0.023;
    this->boh_taxels[20].vector.y = -0.019;
    this->boh_taxels[20].vector.z = -0.016;
    this->boh_taxels[20].header.frame_id = "boh_bracket";
    this->boh_taxels[21].vector.x = -0.032;
    this->boh_taxels[21].vector.y = -0.019;
    this->boh_taxels[21].vector.z = -0.016;
    this->boh_taxels[21].header.frame_id = "boh_bracket";
    this->boh_taxels[22].vector.x = -0.041;
    this->boh_taxels[22].vector.y = -0.019;
    this->boh_taxels[22].vector.z = -0.016;
    this->boh_taxels[22].header.frame_id = "boh_bracket";
    this->boh_taxels[23].vector.x = -0.05;
    this->boh_taxels[23].vector.y = -0.019;
    this->boh_taxels[23].vector.z = -0.016;
    this->boh_taxels[23].header.frame_id = "boh_bracket";
    //Row3
    this->boh_taxels[25].vector.x = 0.047;
    this->boh_taxels[25].vector.y = -0.019;
    this->boh_taxels[25].vector.z = -0.023;
    this->boh_taxels[25].header.frame_id = "boh_bracket";
    this->boh_taxels[26].vector.x = 0.038;
    this->boh_taxels[26].vector.y = -0.019;
    this->boh_taxels[26].vector.z = -0.023;
    this->boh_taxels[26].header.frame_id = "boh_bracket";
    this->boh_taxels[27].vector.x = 0.03;
    this->boh_taxels[27].vector.y = -0.019;
    this->boh_taxels[27].vector.z = -0.023;
    this->boh_taxels[27].header.frame_id = "boh_bracket";
    this->boh_taxels[28].vector.x = 0.021;
    this->boh_taxels[28].vector.y = -0.019;
    this->boh_taxels[28].vector.z = -0.023;
    this->boh_taxels[28].header.frame_id = "boh_bracket";
    this->boh_taxels[29].vector.x = 0.013;
    this->boh_taxels[29].vector.y = -0.019;
    this->boh_taxels[29].vector.z = -0.023;
    this->boh_taxels[29].header.frame_id = "boh_bracket";
    this->boh_taxels[30].vector.x = 0.004;
    this->boh_taxels[30].vector.y = -0.019;
    this->boh_taxels[30].vector.z = -0.023;
    this->boh_taxels[30].header.frame_id = "boh_bracket";
    this->boh_taxels[31].vector.x = -0.005;
    this->boh_taxels[31].vector.y = -0.019;
    this->boh_taxels[31].vector.z = -0.023;
    this->boh_taxels[31].header.frame_id = "boh_bracket";
    this->boh_taxels[32].vector.x = -0.014;
    this->boh_taxels[32].vector.y = -0.019;
    this->boh_taxels[32].vector.z = -0.023;
    this->boh_taxels[32].header.frame_id = "boh_bracket";
    this->boh_taxels[33].vector.x = -0.023;
    this->boh_taxels[33].vector.y = -0.019;
    this->boh_taxels[33].vector.z = -0.023;
    this->boh_taxels[33].header.frame_id = "boh_bracket";
    this->boh_taxels[34].vector.x = -0.032;
    this->boh_taxels[34].vector.y = -0.019;
    this->boh_taxels[34].vector.z = -0.023;
    this->boh_taxels[34].header.frame_id = "boh_bracket";
    this->boh_taxels[35].vector.x = -0.041;
    this->boh_taxels[35].vector.y = -0.019;
    this->boh_taxels[35].vector.z = -0.023;
    this->boh_taxels[35].header.frame_id = "boh_bracket";
    this->boh_taxels[36].vector.x = -0.05;
    this->boh_taxels[36].vector.y = -0.019;
    this->boh_taxels[36].vector.z = -0.023;
    this->boh_taxels[36].header.frame_id = "boh_bracket";
    //Row4
    this->boh_taxels[37].vector.x = 0.047;
    this->boh_taxels[37].vector.y = -0.019;
    this->boh_taxels[37].vector.z = -0.03;
    this->boh_taxels[37].header.frame_id = "boh_bracket";
    this->boh_taxels[38].vector.x = 0.038;
    this->boh_taxels[38].vector.y = -0.019;
    this->boh_taxels[38].vector.z = -0.03;
    this->boh_taxels[38].header.frame_id = "boh_bracket";
    this->boh_taxels[39].vector.x = 0.03;
    this->boh_taxels[39].vector.y = -0.019;
    this->boh_taxels[39].vector.z = -0.03;
    this->boh_taxels[39].header.frame_id = "boh_bracket";
    this->boh_taxels[40].vector.x = 0.021;
    this->boh_taxels[40].vector.y = -0.019;
    this->boh_taxels[40].vector.z = -0.03;
    this->boh_taxels[40].header.frame_id = "boh_bracket";
    this->boh_taxels[41].vector.x = 0.013;
    this->boh_taxels[41].vector.y = -0.019;
    this->boh_taxels[41].vector.z = -0.03;
    this->boh_taxels[41].header.frame_id = "boh_bracket";
    this->boh_taxels[42].vector.x = 0.004;
    this->boh_taxels[42].vector.y = -0.019;
    this->boh_taxels[42].vector.z = -0.03;
    this->boh_taxels[42].header.frame_id = "boh_bracket";
    this->boh_taxels[43].vector.x = -0.005;
    this->boh_taxels[43].vector.y = -0.019;
    this->boh_taxels[43].vector.z = -0.03;
    this->boh_taxels[43].header.frame_id = "boh_bracket";
    this->boh_taxels[44].vector.x = -0.014;
    this->boh_taxels[44].vector.y = -0.019;
    this->boh_taxels[44].vector.z = -0.03;
    this->boh_taxels[44].header.frame_id = "boh_bracket";
    this->boh_taxels[45].vector.x = -0.023;
    this->boh_taxels[45].vector.y = -0.019;
    this->boh_taxels[45].vector.z = -0.03;
    this->boh_taxels[45].header.frame_id = "boh_bracket";
    this->boh_taxels[46].vector.x = -0.032;
    this->boh_taxels[46].vector.y = -0.019;
    this->boh_taxels[46].vector.z = -0.03;
    this->boh_taxels[46].header.frame_id = "boh_bracket";
    this->boh_taxels[47].vector.x = -0.041;
    this->boh_taxels[47].vector.y = -0.019;
    this->boh_taxels[47].vector.z = -0.03;
    this->boh_taxels[47].header.frame_id = "boh_bracket";
    this->boh_taxels[48].vector.x = -0.05;
    this->boh_taxels[48].vector.y = -0.019;
    this->boh_taxels[48].vector.z = -0.03;
    this->boh_taxels[48].header.frame_id = "boh_bracket";
    //Row5
    this->boh_taxels[49].vector.x = 0.047;
    this->boh_taxels[49].vector.y = -0.019;
    this->boh_taxels[49].vector.z = -0.037;
    this->boh_taxels[49].header.frame_id = "boh_bracket";
    this->boh_taxels[50].vector.x = 0.038;
    this->boh_taxels[50].vector.y = -0.019;
    this->boh_taxels[50].vector.z = -0.037;
    this->boh_taxels[50].header.frame_id = "boh_bracket";
    this->boh_taxels[51].vector.x = 0.03;
    this->boh_taxels[51].vector.y = -0.019;
    this->boh_taxels[51].vector.z = -0.037;
    this->boh_taxels[51].header.frame_id = "boh_bracket";
    this->boh_taxels[52].vector.x = 0.021;
    this->boh_taxels[52].vector.y = -0.019;
    this->boh_taxels[52].vector.z = -0.037;
    this->boh_taxels[52].header.frame_id = "boh_bracket";
    this->boh_taxels[53].vector.x = 0.013;
    this->boh_taxels[53].vector.y = -0.019;
    this->boh_taxels[53].vector.z = -0.037;
    this->boh_taxels[53].header.frame_id = "boh_bracket";
    this->boh_taxels[54].vector.x = 0.004;
    this->boh_taxels[54].vector.y = -0.019;
    this->boh_taxels[54].vector.z = -0.037;
    this->boh_taxels[54].header.frame_id = "boh_bracket";
    this->boh_taxels[55].vector.x = -0.005;
    this->boh_taxels[55].vector.y = -0.019;
    this->boh_taxels[55].vector.z = -0.037;
    this->boh_taxels[55].header.frame_id = "boh_bracket";
    this->boh_taxels[56].vector.x = -0.014;
    this->boh_taxels[56].vector.y = -0.019;
    this->boh_taxels[56].vector.z = -0.037;
    this->boh_taxels[56].header.frame_id = "boh_bracket";
    this->boh_taxels[57].vector.x = -0.023;
    this->boh_taxels[57].vector.y = -0.019;
    this->boh_taxels[57].vector.z = -0.037;
    this->boh_taxels[57].header.frame_id = "boh_bracket";
    this->boh_taxels[58].vector.x = -0.032;
    this->boh_taxels[58].vector.y = -0.019;
    this->boh_taxels[58].vector.z = -0.037;
    this->boh_taxels[58].header.frame_id = "boh_bracket";
    this->boh_taxels[59].vector.x = -0.041;
    this->boh_taxels[59].vector.y = -0.019;
    this->boh_taxels[59].vector.z = -0.037;
    this->boh_taxels[59].header.frame_id = "boh_bracket";
    this->boh_taxels[60].vector.x = -0.05;
    this->boh_taxels[60].vector.y = -0.019;
    this->boh_taxels[60].vector.z = -0.037;
    this->boh_taxels[60].header.frame_id = "boh_bracket";
    //Row6
    this->boh_taxels[61].vector.x = 0.047;
    this->boh_taxels[61].vector.y = -0.019;
    this->boh_taxels[61].vector.z = -0.044;
    this->boh_taxels[61].header.frame_id = "boh_bracket";
    this->boh_taxels[62].vector.x = 0.038;
    this->boh_taxels[62].vector.y = -0.019;
    this->boh_taxels[62].vector.z = -0.044;
    this->boh_taxels[62].header.frame_id = "boh_bracket";
    this->boh_taxels[63].vector.x = 0.03;
    this->boh_taxels[63].vector.y = -0.019;
    this->boh_taxels[63].vector.z = -0.044;
    this->boh_taxels[63].header.frame_id = "boh_bracket";
    this->boh_taxels[64].vector.x = 0.021;
    this->boh_taxels[64].vector.y = -0.019;
    this->boh_taxels[64].vector.z = -0.044;
    this->boh_taxels[64].header.frame_id = "boh_bracket";
    this->boh_taxels[65].vector.x = 0.013;
    this->boh_taxels[65].vector.y = -0.019;
    this->boh_taxels[65].vector.z = -0.044;
    this->boh_taxels[65].header.frame_id = "boh_bracket";
    this->boh_taxels[66].vector.x = 0.004;
    this->boh_taxels[66].vector.y = -0.019;
    this->boh_taxels[66].vector.z = -0.044;
    this->boh_taxels[66].header.frame_id = "boh_bracket";
    this->boh_taxels[67].vector.x = -0.005;
    this->boh_taxels[67].vector.y = -0.019;
    this->boh_taxels[67].vector.z = -0.044;
    this->boh_taxels[67].header.frame_id = "boh_bracket";
    this->boh_taxels[68].vector.x = -0.014;
    this->boh_taxels[68].vector.y = -0.019;
    this->boh_taxels[68].vector.z = -0.044;
    this->boh_taxels[68].header.frame_id = "boh_bracket";
    this->boh_taxels[69].vector.x = -0.023;
    this->boh_taxels[69].vector.y = -0.019;
    this->boh_taxels[69].vector.z = -0.044;
    this->boh_taxels[69].header.frame_id = "boh_bracket";
    this->boh_taxels[70].vector.x = -0.032;
    this->boh_taxels[70].vector.y = -0.019;
    this->boh_taxels[70].vector.z = -0.044;
    this->boh_taxels[70].header.frame_id = "boh_bracket";
    this->boh_taxels[71].vector.x = -0.041;
    this->boh_taxels[71].vector.y = -0.019;
    this->boh_taxels[71].vector.z = -0.044;
    this->boh_taxels[71].header.frame_id = "boh_bracket";
    this->boh_taxels[72].vector.x = -0.05;
    this->boh_taxels[72].vector.y = -0.019;
    this->boh_taxels[72].vector.z = -0.044;
    this->boh_taxels[72].header.frame_id = "boh_bracket";
    //Row7
    this->boh_taxels[73].vector.x = 0.047;
    this->boh_taxels[73].vector.y = -0.019;
    this->boh_taxels[73].vector.z = -0.051;
    this->boh_taxels[73].header.frame_id = "boh_bracket";
    this->boh_taxels[74].vector.x = 0.038;
    this->boh_taxels[74].vector.y = -0.019;
    this->boh_taxels[74].vector.z = -0.051;
    this->boh_taxels[74].header.frame_id = "boh_bracket";
    this->boh_taxels[75].vector.x = 0.03;
    this->boh_taxels[75].vector.y = -0.019;
    this->boh_taxels[75].vector.z = -0.051;
    this->boh_taxels[75].header.frame_id = "boh_bracket";
    this->boh_taxels[76].vector.x = 0.021;
    this->boh_taxels[76].vector.y = -0.019;
    this->boh_taxels[76].vector.z = -0.051;
    this->boh_taxels[76].header.frame_id = "boh_bracket";
    this->boh_taxels[77].vector.x = 0.013;
    this->boh_taxels[77].vector.y = -0.019;
    this->boh_taxels[77].vector.z = -0.051;
    this->boh_taxels[77].header.frame_id = "boh_bracket";
    this->boh_taxels[78].vector.x = 0.004;
    this->boh_taxels[78].vector.y = -0.019;
    this->boh_taxels[78].vector.z = -0.051;
    this->boh_taxels[78].header.frame_id = "boh_bracket";
    this->boh_taxels[79].vector.x = -0.005;
    this->boh_taxels[79].vector.y = -0.019;
    this->boh_taxels[79].vector.z = -0.051;
    this->boh_taxels[79].header.frame_id = "boh_bracket";
    this->boh_taxels[80].vector.x = -0.014;
    this->boh_taxels[80].vector.y = -0.019;
    this->boh_taxels[80].vector.z = -0.051;
    this->boh_taxels[80].header.frame_id = "boh_bracket";
    this->boh_taxels[81].vector.x = -0.023;
    this->boh_taxels[81].vector.y = -0.019;
    this->boh_taxels[81].vector.z = -0.051;
    this->boh_taxels[81].header.frame_id = "boh_bracket";
    this->boh_taxels[82].vector.x = -0.032;
    this->boh_taxels[82].vector.y = -0.019;
    this->boh_taxels[82].vector.z = -0.051;
    this->boh_taxels[82].header.frame_id = "boh_bracket";
    this->boh_taxels[83].vector.x = -0.041;
    this->boh_taxels[83].vector.y = -0.019;
    this->boh_taxels[83].vector.z = -0.051;
    this->boh_taxels[83].header.frame_id = "boh_bracket";
    this->boh_taxels[84].vector.x = -0.05;
    this->boh_taxels[84].vector.y = -0.019;
    this->boh_taxels[84].vector.z = -0.051;
    this->boh_taxels[84].header.frame_id = "boh_bracket";
    //Row8
    this->boh_taxels[85].vector.x = 0.047;
    this->boh_taxels[85].vector.y = -0.019;
    this->boh_taxels[85].vector.z = -0.058;
    this->boh_taxels[85].header.frame_id = "boh_bracket";
    this->boh_taxels[86].vector.x = 0.038;
    this->boh_taxels[86].vector.y = -0.019;
    this->boh_taxels[86].vector.z = -0.058;
    this->boh_taxels[86].header.frame_id = "boh_bracket";
    this->boh_taxels[87].vector.x = 0.03;
    this->boh_taxels[87].vector.y = -0.019;
    this->boh_taxels[87].vector.z = -0.058;
    this->boh_taxels[87].header.frame_id = "boh_bracket";
    this->boh_taxels[88].vector.x = 0.021;
    this->boh_taxels[88].vector.y = -0.019;
    this->boh_taxels[88].vector.z = -0.058;
    this->boh_taxels[88].header.frame_id = "boh_bracket";
    this->boh_taxels[89].vector.x = 0.013;
    this->boh_taxels[89].vector.y = -0.019;
    this->boh_taxels[89].vector.z = -0.058;
    this->boh_taxels[89].header.frame_id = "boh_bracket";
    this->boh_taxels[90].vector.x = 0.004;
    this->boh_taxels[90].vector.y = -0.019;
    this->boh_taxels[90].vector.z = -0.058;
    this->boh_taxels[90].header.frame_id = "boh_bracket";
    this->boh_taxels[91].vector.x = -0.005;
    this->boh_taxels[91].vector.y = -0.019;
    this->boh_taxels[91].vector.z = -0.058;
    this->boh_taxels[91].header.frame_id = "boh_bracket";
    this->boh_taxels[92].vector.x = -0.014;
    this->boh_taxels[92].vector.y = -0.019;
    this->boh_taxels[92].vector.z = -0.058;
    this->boh_taxels[92].header.frame_id = "boh_bracket";
    this->boh_taxels[93].vector.x = -0.023;
    this->boh_taxels[93].vector.y = -0.019;
    this->boh_taxels[93].vector.z = -0.058;
    this->boh_taxels[93].header.frame_id = "boh_bracket";
    this->boh_taxels[94].vector.x = -0.032;
    this->boh_taxels[94].vector.y = -0.019;
    this->boh_taxels[94].vector.z = -0.058;
    this->boh_taxels[94].header.frame_id = "boh_bracket";
    this->boh_taxels[95].vector.x = -0.041;
    this->boh_taxels[95].vector.y = -0.019;
    this->boh_taxels[95].vector.z = -0.058;
    this->boh_taxels[95].header.frame_id = "boh_bracket";
    this->boh_taxels[96].vector.x = -0.05;
    this->boh_taxels[96].vector.y = -0.019;
    this->boh_taxels[96].vector.z = -0.058;
    this->boh_taxels[96].header.frame_id = "boh_bracket";
    //Row9
    this->boh_taxels[97].vector.x = 0.047;
    this->boh_taxels[97].vector.y = -0.019;
    this->boh_taxels[97].vector.z = -0.065;
    this->boh_taxels[97].header.frame_id = "boh_bracket";
    this->boh_taxels[98].vector.x = 0.038;
    this->boh_taxels[98].vector.y = -0.019;
    this->boh_taxels[98].vector.z = -0.065;
    this->boh_taxels[98].header.frame_id = "boh_bracket";
    this->boh_taxels[99].vector.x = 0.03;
    this->boh_taxels[99].vector.y = -0.019;
    this->boh_taxels[99].vector.z = -0.065;
    this->boh_taxels[99].header.frame_id = "boh_bracket";
    this->boh_taxels[100].vector.x = 0.021;
    this->boh_taxels[100].vector.y = -0.019;
    this->boh_taxels[100].vector.z = -0.065;
    this->boh_taxels[100].header.frame_id = "boh_bracket";
    this->boh_taxels[101].vector.x = 0.013;
    this->boh_taxels[101].vector.y = -0.019;
    this->boh_taxels[101].vector.z = -0.065;
    this->boh_taxels[101].header.frame_id = "boh_bracket";
    this->boh_taxels[102].vector.x = 0.004;
    this->boh_taxels[102].vector.y = -0.019;
    this->boh_taxels[102].vector.z = -0.065;
    this->boh_taxels[102].header.frame_id = "boh_bracket";
    this->boh_taxels[103].vector.x = -0.005;
    this->boh_taxels[103].vector.y = -0.019;
    this->boh_taxels[103].vector.z = -0.065;
    this->boh_taxels[103].header.frame_id = "boh_bracket";
    this->boh_taxels[104].vector.x = -0.014;
    this->boh_taxels[104].vector.y = -0.019;
    this->boh_taxels[104].vector.z = -0.065;
    this->boh_taxels[104].header.frame_id = "boh_bracket";
    this->boh_taxels[105].vector.x = -0.023;
    this->boh_taxels[105].vector.y = -0.019;
    this->boh_taxels[105].vector.z = -0.065;
    this->boh_taxels[105].header.frame_id = "boh_bracket";
    this->boh_taxels[106].vector.x = -0.032;
    this->boh_taxels[106].vector.y = -0.019;
    this->boh_taxels[106].vector.z = -0.065;
    this->boh_taxels[106].header.frame_id = "boh_bracket";
    this->boh_taxels[107].vector.x = -0.041;
    this->boh_taxels[107].vector.y = -0.019;
    this->boh_taxels[107].vector.z = -0.065;
    this->boh_taxels[107].header.frame_id = "boh_bracket";
    this->boh_taxels[108].vector.x = -0.05;
    this->boh_taxels[108].vector.y = -0.019;
    this->boh_taxels[108].vector.z = -0.065;
    this->boh_taxels[108].header.frame_id = "boh_bracket";


    // Filling the taxels_map
    this->taxels_map["boh"] = this->boh_taxels;
    this->taxels_map["palm"] = this->palm_taxels;
    this->taxels_map["ipb"] = this->ipb_taxels;
    this->taxels_map["mpb"] = this->mpb_taxels;
    this->taxels_map["ppb"] = this->ppb_taxels;
    this->taxels_map["mpf"] = this->mpf_taxels;
    this->taxels_map["ppf"] = this->ppf_taxels;
    this->taxels_map["tmb"] = this->tmb_taxels;
    this->taxels_map["imb"] = this->imb_taxels;
    this->taxels_map["mmb"] = this->mmb_taxels;
    this->taxels_map["pmb"] = this->pmb_taxels;
    this->taxels_map["tmf"] = this->tmf_taxels;
    this->taxels_map["imf"] = this->imf_taxels;
    this->taxels_map["mmf"] = this->mmf_taxels;
    this->taxels_map["pmf"] = this->pmf_taxels;
    this->taxels_map["tft"] = this->tft_taxels;
    this->taxels_map["ift"] = this->ift_taxels;
    this->taxels_map["mft"] = this->mft_taxels;
    this->taxels_map["pft"] = this->pft_taxels;
//Preparing the filtered taxels_map
    // Filling the transformed taxels_map
    this->transformed_taxels_map["boh"] = this->trans_boh_taxels;
    this->transformed_taxels_map["palm"] = this->trans_palm_taxels;
    this->transformed_taxels_map["ipb"] = this->trans_ipb_taxels;
    this->transformed_taxels_map["mpb"] = this->trans_mpb_taxels;
    this->transformed_taxels_map["ppb"] = this->trans_ppb_taxels;
    this->transformed_taxels_map["mpf"] = this->trans_mpf_taxels;
    this->transformed_taxels_map["ppf"] = this->trans_ppf_taxels;
    this->transformed_taxels_map["tmb"] = this->trans_tmb_taxels;
    this->transformed_taxels_map["imb"] = this->trans_imb_taxels;
    this->transformed_taxels_map["mmb"] = this->trans_mmb_taxels;
    this->transformed_taxels_map["pmb"] = this->trans_pmb_taxels;
    this->transformed_taxels_map["tmf"] = this->trans_tmf_taxels;
    this->transformed_taxels_map["imf"] = this->trans_imf_taxels;
    this->transformed_taxels_map["mmf"] = this->trans_mmf_taxels;
    this->transformed_taxels_map["pmf"] = this->trans_pmf_taxels;
    this->transformed_taxels_map["tft"] = this->trans_tft_taxels;
    this->transformed_taxels_map["ift"] = this->trans_ift_taxels;
    this->transformed_taxels_map["mft"] = this->trans_mft_taxels;
    this->transformed_taxels_map["pft"] = this->trans_pft_taxels;


}


// Called by the world update start event
void CoordinatesPlugin::OnUpdate()
{

}

/// \brief ROS helper function that processes messages
void CoordinatesPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
// ROS saving subscriber callback function
void CoordinatesPlugin::saving_callback(const std_msgs::StringConstPtr &_msg) //
{
    this->act_cmd = _msg->data;
}

// Gazebo contact detection subscriber callback
void CoordinatesPlugin::contact_callback(ContactPtr &_msg)
{
    bool new_contact_pair = true;
    std::string sensor_name;
    std::pair<std::string, std::string> contact_pair;
    std::vector<std::pair<std::string, std::string>> contact_pair_v;
    std::vector<float> contact_pt_v;
    //Vector resizing
    contact_pt_v.resize(3);
    if (_msg->contact_size() > 0)
    {

        //std::cout<< "Contact Size: " << _msg->contact_size() << std::endl;
        //std::cout<< _msg->DebugString() << std::endl;

        for (unsigned int i = 0; i < _msg->contact_size(); ++i)
        {
            std::pair<std::string, std::string> current_contact_pair =
                 std::make_pair( _msg->contact(i).collision1(), _msg->contact(i).collision2());
            //Taking into account the reversed current_contact_pair as well
            std::pair<std::string, std::string> rev_current_contact_pair =
                 std::make_pair( _msg->contact(i).collision2(), _msg->contact(i).collision1());
            if (i==0)
            {
                contact_pair_v.push_back(current_contact_pair);
                //Storing the reversed current_contact_pair as well
                contact_pair_v.push_back(rev_current_contact_pair);
            }
            else if (std::find(contact_pair_v.begin(),contact_pair_v.end(),
                     current_contact_pair) != contact_pair_v.end())
            {
               new_contact_pair = false;
            }
            else
            {
                new_contact_pair = true;
                //Adding the new contact pair to the vector of contacts
                contact_pair_v.push_back(current_contact_pair);
                //Storing the reversed current_contact_pair as well
                contact_pair_v.push_back(rev_current_contact_pair);
            }

            if(new_contact_pair)
            {
                  //std::cout << _msg->contact_size() << std::endl;
                  //std::cout << "Collision1: " << _msg->contact(i).collision1()<< "   "
                  //          << "Collision2: " << _msg->contact(i).collision2() <<std::endl;
                //***********************************************************
                //Registering the sensor's name
                //***********************************************************
                if(_msg->contact(i).collision1().find("_bracket")!=std::string::npos)
                {
                    sensor_name = this->getName(_msg->contact(i).collision1());
                }
                else if(_msg->contact(i).collision2().find("_bracket")!=std::string::npos)
                {
                    sensor_name = this->getName(_msg->contact(i).collision2());
                }

                //std::cout<< "Sensor in contact: " << sensor_name<< std::endl;

                for (unsigned int j = 0; j < _msg->contact(i).position_size(); ++j)
                {
                    //Registering each contact point coordinates for the current contact message
                    contact_pt_v[0] = _msg->contact(i).position(j).x();
                    contact_pt_v[1] = _msg->contact(i).position(j).y();
                    contact_pt_v[2] = _msg->contact(i).position(j).z();
                    //std::cout << contact_pt_v[0] << " " <<
                    //             contact_pt_v[1] << " " <<
                    //             contact_pt_v[2] << " " << std::endl;
                    //Saving the contact point to the contact_positions vector
                    this->contact_positions_v.push_back(contact_pt_v);

                }
            }

         }

        //Updating the raw contact map
        this->raw_contacts_map[sensor_name] = this->contact_positions_v;
        //std::cout<< "Sensor in contact: " << sensor_name << std::endl;
        //std::cout << "Number of contact points: " << this->contact_positions_v.size() << std::endl;
        //for(unsigned int j; j < this->contact_positions_v.size(); ++j)
        //{
        //    std::cout << "  Contact Position: " << j << " "
        //              << this->contact_positions_v[j][0] << " "
        //              << this->contact_positions_v[j][1] << " "
        //              << this->contact_positions_v[j][2] << "\n";
        //
        //Contact positions vector clearance
        this->contact_positions_v.clear();
    }
    if(this->act_cmd == "true")
    {

        //Filtering of the raw_contacts_map
        for (auto const& [key, val] : this->raw_contacts_map) // key is the sensor name: ift, mpb...
                                                              // val is a vector of contact locations for the correspondent sensor
        {
           //this->filtered_contact_positions_v.push_back(std::vector<float>{0,0,0});
           this->target_sensor = key;
           // Getting the transformation from the brackets to the world frame using tf2
           // For now, focus only on "boh_bracket"
           if(tfBuffer.canTransform("world",key+"_bracket", ros::Time(0)))
           {
               this->transformStamped = tfBuffer.lookupTransform("world",key+"_bracket", ros::Time(0));
               for (unsigned int i = 0; i < this->taxels_map[key].size() ; ++i)
               {

                   /*
                   std::cout<<"Taxel " <<std::to_string(i) << " before trans coordinates: "
                            <<"X: "<<this->taxels_map[key][i].vector.x
                            <<" Y: " << this->taxels_map[key][i].vector.y
                            << " Z: "<< this->taxels_map[key][i].vector.z << std::endl;
                   */

                   this->transformed_taxels_map[key][i] = this->homotrans(this->transformStamped,taxels_map[key][i]);// Apply the transformations on "taxels_map[key]"

                   /*
                   std::cout<<"Taxel " <<std::to_string(i) << " after trans coordinates: "
                            <<"X: "<<this->transformed_taxels_map[key][i][0]
                            <<" Y: " << this->transformed_taxels_map[key][i][1]
                            << " Z: "<< this->transformed_taxels_map[key][i][2] << std::endl;
                   */


               }
           }
           //********************************Testing Area************************************************
           for (unsigned int i = 0; i < val.size() ; ++i)
           {
               //Comparing the location of each contact point of a sensor with the locations of the same sensor's taxels
               for (unsigned int j = 0; j < this->transformed_taxels_map[key].size() ; ++j)
               {
                   std::cout << "Distance: "
                             << this->dist(val[i],this->transformed_taxels_map[key][j])
                             << std::endl;
                   if(this->dist(val[i],this->transformed_taxels_map[key][j]) < dist_threshold &&
                           std::find(this->filtered_contact_positions_v.begin(),this->filtered_contact_positions_v.end(),
                           this->transformed_taxels_map[key][j]) == this->filtered_contact_positions_v.end())
                   {
                       //Replace the contact point by the correspondent taxel(s)
                       std::cout<<"Pushed contact vector: " << transformed_taxels_map[key][j][0]
                                << transformed_taxels_map[key][j][1] << transformed_taxels_map[key][j][2]
                                << std::endl;
                       this->filtered_contact_positions_v.push_back(this->transformed_taxels_map[key][j]);
                   }
               }
           }
           this->filtered_contacts_map[key] = this->filtered_contact_positions_v;
           this->filtered_contact_positions_v.clear();
        }
        //Flushing the contact locations into a .csv file
        for (auto const& [key, val] : this->raw_contacts_map)
        {

            if(this->fp == NULL) {
                printf("file can't be opened\n");
                exit(1);
            }
            fflush(stdin);
            for (unsigned int i = 0; i < val.size() ; ++i)
            {
            fprintf(this->fp, "%f,%f,%f \n", val[i][0],val[i][1],val[i][2]);
            }

        }
        //Flushing the FILTERED contact locations into a .csv file
        for (auto const& [key, val] : this->filtered_contacts_map)
        {

            if(this->fp_filtered == NULL) {
                printf("file can't be opened\n");
                exit(1);
            }
            fflush(stdin);
            for (unsigned int i = 0; i < val.size() ; ++i)
            {
            fprintf(this->fp_filtered, "%f,%f,%f \n", val[i][0],val[i][1],val[i][2]);
            }

        }

    fclose(this->fp);
    fclose(this->fp_filtered);
    this->act_cmd = "false";
    ros::shutdown();
    }

}

void CoordinatesPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CoordinatesPlugin::OnUpdate, this));
      //Gazebo transport node creation and initialization
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(_parent->GetName());
      this->sensorManager = sensors::SensorManager::Instance();
      // The total number of attached contact sensors
      std::cout << "The number of attached sensors is: " + std::to_string(_parent->GetSensorCount()) << std::endl;
      //Create a subscriber for each contact sensor that was placed on each FPCB and register a common callback
      for (int index = 0; index < _parent->GetSensorCount(); index++) {
          //Vectors sizing to avoid memory violation
          this->sensor_v.resize(_parent->GetSensorCount());
          this->subscribers_v.resize(_parent->GetSensorCount());

          this->sensor_v = this->sensorManager->GetSensors();
          std::string topic_name = sensor_v[index]->ScopedName();
          boost::replace_all(topic_name, "::", "/");
          std::cout << "/gazebo/"+topic_name << std::endl;
          this->subscribers_v[index] = this->node->Subscribe("/gazebo/"+topic_name,&CoordinatesPlugin::contact_callback,this);
          }
          //this->saving_subscriber = this->node->Subscribe("saving_order",&CoordinatesPlugin::saving_callback,this);
          //Opening the .csv file for contact points registration
          //std::string full_path = std::filesystem::current_path();
          //size_t pos = full_path.find("Gazebo_Simulation");
          //std::string path = full_path.substr(0,pos+17)+"/Pt_Cloud_Scripts/contacts_coordinates.csv";
          //std::string path_filtered = full_path.substr(0,pos+17)+"/Pt_Cloud_Scripts/filtered_contacts_coordinates.csv";
          std::string path = "contacts_coordinates.csv";
          std::string path_filtered = "filtered_contacts_coordinates.csv";
          this->fp = fopen(path.c_str(), "w");
          this->fp_filtered = fopen(path_filtered.c_str(), "w");
          //Building the taxels data structure
          this->taxelsTreeBld();

          //ROS subscriber
          // Initialize ros, if it has not already bee initialized.
          if (!ros::isInitialized())
          {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
                ros::init_options::NoSigintHandler);
          }
          // Create the tf listener
          tfListener = new tf2_ros::TransformListener(tfBuffer);

          // Create our ROS node. This acts in a similar manner to
          // the Gazebo node
          this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

          // Create a named topic, and subscribe to it.
          ros::SubscribeOptions so =
            ros::SubscribeOptions::create<std_msgs::String>(
                      "/saving_order",
                      5,
                      boost::bind(&CoordinatesPlugin::saving_callback, this, _1),
                      ros::VoidPtr(),
                      &this->rosQueue);
          this->saving_subscriber = this->rosNode->subscribe(so);

          // Spin up the queue helper thread.
          this->rosQueueThread =
            std::thread(std::bind(&CoordinatesPlugin::QueueThread, this));
          this->rosQueueThread.detach();


    }




//*****************************************************Test******************************************




