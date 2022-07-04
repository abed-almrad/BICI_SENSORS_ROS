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
//Method to adjust the positions of the raw contact points
std::vector<float> CoordinatesPlugin::rawhomotrans(geometry_msgs::TransformStamped transformStamped_1,
                                                   geometry_msgs::TransformStamped transformStamped_2,
                                                   geometry_msgs::Vector3Stamped contact_pt)
{
    Eigen::Matrix4d resultant_matrix;
    Eigen::Vector4d resultant_vector;
    Eigen::Vector4d homogeneous_vector(contact_pt.vector.x,contact_pt.vector.y,contact_pt.vector.z,1.0);
    std::vector<std::vector<float>> homogeneous_matrix_1,homogeneous_matrix_2;
    std::vector<float> result;
    result.resize(3);

    std::cout << "Homogeneous vector: " << homogeneous_vector[0] << " " << homogeneous_vector[1]
              << " " << homogeneous_vector[2] <<" " << homogeneous_vector[3] << std::endl;

    homogeneous_matrix_1 = this->homo_matrix(transformStamped_1);

    std::cout << "Homogeneous matrix 1:\n"
              << homogeneous_matrix_1[0][0] << " " << homogeneous_matrix_1[0][1]
              << " " << homogeneous_matrix_1[0][2] << " " << homogeneous_matrix_1[0][3] << "\n"
              << homogeneous_matrix_1[1][0] << " " << homogeneous_matrix_1[1][1]<< " "
              <<homogeneous_matrix_1[1][2] << " " << homogeneous_matrix_1[1][3] << "\n"
              << homogeneous_matrix_1[2][0] << " " << homogeneous_matrix_1[2][1]<< " "
              <<homogeneous_matrix_1[2][2] << " " << homogeneous_matrix_1[2][3] << "\n"
              << homogeneous_matrix_1[3][0] << " " << homogeneous_matrix_1[3][1]<< " "
              <<homogeneous_matrix_1[3][2] << " " << homogeneous_matrix_1[3][3] << "\n"<<std::endl;

    homogeneous_matrix_2 = this->homo_matrix(transformStamped_2);

    std::cout << "Homogeneous matrix 2:\n"
              << homogeneous_matrix_2[0][0] << " " << homogeneous_matrix_2[0][1]
              << " " << homogeneous_matrix_2[0][2] << " " << homogeneous_matrix_2[0][3] << "\n"
              << homogeneous_matrix_2[1][0] << " " << homogeneous_matrix_2[1][1]<< " "
              <<homogeneous_matrix_2[1][2] << " " << homogeneous_matrix_2[1][3] << "\n"
              << homogeneous_matrix_2[2][0] << " " << homogeneous_matrix_2[2][1]<< " "
              <<homogeneous_matrix_2[2][2] << " " << homogeneous_matrix_2[2][3] << "\n"
              << homogeneous_matrix_2[3][0] << " " << homogeneous_matrix_2[3][1]<< " "
              <<homogeneous_matrix_2[3][2] << " " << homogeneous_matrix_2[3][3] << "\n"<<std::endl;

    this->input_matrix_1<< homogeneous_matrix_1[0][0],homogeneous_matrix_1[0][1],
                         homogeneous_matrix_1[0][2],homogeneous_matrix_1[0][3],
                         homogeneous_matrix_1[1][0],homogeneous_matrix_1[1][1],
                         homogeneous_matrix_1[1][2],homogeneous_matrix_1[1][3],
                         homogeneous_matrix_1[2][0],homogeneous_matrix_1[2][1],
                         homogeneous_matrix_1[2][2],homogeneous_matrix_1[2][3],
                         homogeneous_matrix_1[3][0],homogeneous_matrix_1[3][1],
                         homogeneous_matrix_1[3][2],homogeneous_matrix_1[3][3];
    std::cout<< "Input matrix 1:\n" << this->input_matrix_1.block(0,0,4,4) << std::endl;

    this->input_matrix_2<< homogeneous_matrix_2[0][0],homogeneous_matrix_2[0][1],
                         homogeneous_matrix_2[0][2],homogeneous_matrix_2[0][3],
                         homogeneous_matrix_2[1][0],homogeneous_matrix_2[1][1],
                         homogeneous_matrix_2[1][2],homogeneous_matrix_2[1][3],
                         homogeneous_matrix_2[2][0],homogeneous_matrix_2[2][1],
                         homogeneous_matrix_2[2][2],homogeneous_matrix_2[2][3],
                         homogeneous_matrix_2[3][0],homogeneous_matrix_2[3][1],
                         homogeneous_matrix_2[3][2],homogeneous_matrix_2[3][3];
    std::cout<< "Input matrix 2:\n" << this->input_matrix_2.block(0,0,4,4) << std::endl;

    if(input_matrix_1.determinant()== 0)
    {
        try
        {
          throw 20;
        }
        catch (int exception)
        {
          std::cout << "An exception occurred. Cannot calculate the inverse of a singular matrix" << std::endl;
        }
    }
    else
    {
        this->output_matrix_1 = this->input_matrix_1.inverse();
        std::cout<< "Inverse of the home pose matrix:\n" << this->output_matrix_1.block(0,0,4,4) << std::endl;
    }
    resultant_matrix = this->input_matrix_2*this->output_matrix_1;
    std::cout<< "Resultant matrix: \n" << resultant_matrix.block(0,0,4,4) << std::endl;
    resultant_vector = resultant_matrix*homogeneous_vector;
    std::cout<< "Resultant homogeneous vector:\n" << resultant_vector[0] <<"  "
                                                  << resultant_vector[1] <<"  "
                                                  << resultant_vector[2] <<"  " << std::endl;
    for (int i=0; i<3;i++)
    {
        result[i] = resultant_vector[i];
    }
    //************************P.S.************************************
    //****************************************************************
    //This last lines of code are making the function temporarely ineffective
    for (int i=0; i<3;i++)
    {
        result[i] = homogeneous_vector[i];
    }
    //std::cout<<"Result: "<< result[0] <<"  "<< result[1] <<"  "<< result[2] << std::endl;
    return result;
}
//Method for matrix multiplication
std::vector<float> CoordinatesPlugin::homotrans(geometry_msgs::TransformStamped transformStamped,geometry_msgs::Vector3Stamped v3stamped)
{
    std::vector<float> homogeneous_vector = this->homo_v(v3stamped);
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
    float q0,q1,q2,q3,dx,dy,dz,norm;
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
    norm = pow(q0,2)+pow(q1,2)+pow(q2,2)+pow(q3,2);
//Wikipedia Method
    homo_matrix[0][0] = 1-2*norm*(pow(q1,2)+pow(q2,2));
    homo_matrix[0][1] = 2*norm*(q0*q1-q2*q3);
    homo_matrix[0][2] = 2*norm*(q0*q2+q1*q3);
    homo_matrix[0][3] = dx;
    homo_matrix[1][0] = 2*norm*(q0*q1+q2*q3);
    homo_matrix[1][1] = 1-2*norm*(pow(q0,2)+pow(q2,2));
    homo_matrix[1][2] = 2*norm*(q1*q2-q0*q3);
    homo_matrix[1][3] = dy;
    homo_matrix[2][0] = 2*norm*(q0*q2-q1*q3);
    homo_matrix[2][1] = 2*norm*(q1*q2+q0*q3);
    homo_matrix[2][2] = 1-2*norm*(pow(q0,2)+pow(q1,2));
    homo_matrix[2][3] = dz;
    homo_matrix[3][0] = 0;
    homo_matrix[3][1] = 0;
    homo_matrix[3][2] = 0;
    homo_matrix[3][3] = 1;

    return homo_matrix;
}
//Method for converting a 3D vector into its homogeneous representation
std::vector<float> CoordinatesPlugin::homo_v(geometry_msgs::Vector3Stamped v3stamped)
{
  std::vector<float> result;
    result.push_back(v3stamped.vector.x);
    result.push_back(v3stamped.vector.y);
    result.push_back(v3stamped.vector.z);
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
    this->boh_taxels.resize(118);
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
    this->trans_boh_taxels.resize(118,std::vector<float>(3,0));
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
    // Specifying the taxels positions w.r.t. to the correspondent brackets frames (i.e. the frames associated with " link_"bracket_name" " in onshape (P.S. Reconsider using push_back instead)
    //*******************************************************************
    //****************************************************************
    //BOH
    //Row1
    this->boh_taxels[0].vector.x = -0.026741;
    this->boh_taxels[0].vector.y = -0.000447;
    this->boh_taxels[0].vector.z = -0.0098;
    this->boh_taxels[1].vector.x = -0.018232;
    this->boh_taxels[1].vector.y = -0.000447;
    this->boh_taxels[1].vector.z = -0.0098;
    this->boh_taxels[2].vector.x = -0.009723;
    this->boh_taxels[2].vector.y = -0.000447;
    this->boh_taxels[2].vector.z = -0.0098;
    this->boh_taxels[3].vector.x = -0.001214;
    this->boh_taxels[3].vector.y = -0.000447;
    this->boh_taxels[3].vector.z = -0.0098;
    this->boh_taxels[4].vector.x = 0.007295;
    this->boh_taxels[4].vector.y = -0.000447;
    this->boh_taxels[4].vector.z = -0.0098;
    this->boh_taxels[5].vector.x = 0.015804;
    this->boh_taxels[5].vector.y = -0.000447;
    this->boh_taxels[5].vector.z = -0.0098;
    this->boh_taxels[6].vector.x = 0.024313;
    this->boh_taxels[6].vector.y = -0.000447;
    this->boh_taxels[6].vector.z = -0.0098;
    this->boh_taxels[7].vector.x = 0.032822;
    this->boh_taxels[7].vector.y = -0.000447;
    this->boh_taxels[7].vector.z = -0.0098;
    this->boh_taxels[8].vector.x = 0.041331;
    this->boh_taxels[8].vector.y = -0.000447;
    this->boh_taxels[8].vector.z = -0.0098;
    this->boh_taxels[9].vector.x = 0.049840;
    this->boh_taxels[9].vector.y = -0.000447;
    this->boh_taxels[9].vector.z = -0.0098;
    this->boh_taxels[10].vector.x = 0.058349;
    this->boh_taxels[10].vector.y = -0.000447;
    this->boh_taxels[10].vector.z = -0.0098;
    this->boh_taxels[11].vector.x = 0.066858;
    this->boh_taxels[11].vector.y = -0.000447;
    this->boh_taxels[11].vector.z = -0.0098;
    //Row2
    this->boh_taxels[12].vector.x = -0.026741;
    this->boh_taxels[12].vector.y = -0.007149;
    this->boh_taxels[12].vector.z = -0.0098;
    this->boh_taxels[13].vector.x = -0.018232;
    this->boh_taxels[13].vector.y = -0.007149;
    this->boh_taxels[13].vector.z = -0.0098;
    this->boh_taxels[14].vector.x = -0.009723;
    this->boh_taxels[14].vector.y = -0.007149;
    this->boh_taxels[14].vector.z = -0.0098;
    this->boh_taxels[15].vector.x = -0.001214;
    this->boh_taxels[15].vector.y = -0.007149;
    this->boh_taxels[15].vector.z = -0.0098;
    this->boh_taxels[16].vector.x = 0.007295;
    this->boh_taxels[16].vector.y = -0.007149;
    this->boh_taxels[16].vector.z = -0.0098;
    this->boh_taxels[17].vector.x = 0.015804;
    this->boh_taxels[17].vector.y = -0.007149;
    this->boh_taxels[17].vector.z = -0.0098;
    this->boh_taxels[18].vector.x = 0.024313;
    this->boh_taxels[18].vector.y = -0.007149;
    this->boh_taxels[18].vector.z = -0.0098;
    this->boh_taxels[19].vector.x = 0.032822;
    this->boh_taxels[19].vector.y = -0.007149;
    this->boh_taxels[19].vector.z = -0.0098;
    this->boh_taxels[20].vector.x = 0.041331;
    this->boh_taxels[20].vector.y = -0.007149;
    this->boh_taxels[20].vector.z = -0.0098;
    this->boh_taxels[21].vector.x = 0.04984;
    this->boh_taxels[21].vector.y = -0.007149;
    this->boh_taxels[21].vector.z = -0.0098;
    this->boh_taxels[22].vector.x = 0.058349;
    this->boh_taxels[22].vector.y = -0.007149;
    this->boh_taxels[22].vector.z = -0.0098;
    this->boh_taxels[23].vector.x = 0.066858;
    this->boh_taxels[23].vector.y = -0.007149;
    this->boh_taxels[23].vector.z = -0.0098;
    //Row3
    this->boh_taxels[24].vector.x = -0.026741;
    this->boh_taxels[24].vector.y = -0.01385;
    this->boh_taxels[24].vector.z = -0.0098;
    this->boh_taxels[25].vector.x = -0.018232;
    this->boh_taxels[25].vector.y = -0.01385;
    this->boh_taxels[25].vector.z = -0.0098;
    this->boh_taxels[26].vector.x = -0.009723;
    this->boh_taxels[26].vector.y = -0.01385;
    this->boh_taxels[26].vector.z = -0.0098;
    this->boh_taxels[27].vector.x = -0.001214;
    this->boh_taxels[27].vector.y = -0.01385;
    this->boh_taxels[27].vector.z = -0.0098;
    this->boh_taxels[28].vector.x = 0.007295;
    this->boh_taxels[28].vector.y = -0.01385;
    this->boh_taxels[28].vector.z = -0.0098;
    this->boh_taxels[29].vector.x = 0.015804;
    this->boh_taxels[29].vector.y = -0.01385;
    this->boh_taxels[29].vector.z = -0.0098;
    this->boh_taxels[30].vector.x = 0.024313;
    this->boh_taxels[30].vector.y = -0.01385;
    this->boh_taxels[30].vector.z = -0.0098;
    this->boh_taxels[31].vector.x = 0.032822;
    this->boh_taxels[31].vector.y = -0.01385;
    this->boh_taxels[31].vector.z = -0.0098;
    this->boh_taxels[32].vector.x = 0.041331;
    this->boh_taxels[32].vector.y = -0.01385;
    this->boh_taxels[32].vector.z = -0.0098;
    this->boh_taxels[33].vector.x = 0.04984;
    this->boh_taxels[33].vector.y = -0.01385;
    this->boh_taxels[33].vector.z = -0.0098;
    this->boh_taxels[34].vector.x = 0.058349;
    this->boh_taxels[34].vector.y = -0.01385;
    this->boh_taxels[34].vector.z = -0.0098;
    this->boh_taxels[35].vector.x = 0.066858;
    this->boh_taxels[35].vector.y = -0.01385;
    this->boh_taxels[35].vector.z = -0.0098;
    //Row4
    this->boh_taxels[36].vector.x = -0.026741;
    this->boh_taxels[36].vector.y = -0.020551;
    this->boh_taxels[36].vector.z = -0.0098;
    this->boh_taxels[37].vector.x = -0.018232;
    this->boh_taxels[37].vector.y = -0.020551;
    this->boh_taxels[37].vector.z = -0.0098;
    this->boh_taxels[38].vector.x = -0.009723;
    this->boh_taxels[38].vector.y = -0.020551;
    this->boh_taxels[38].vector.z = -0.0098;
    this->boh_taxels[39].vector.x = -0.001214;
    this->boh_taxels[39].vector.y = -0.020551;
    this->boh_taxels[39].vector.z = -0.0098;
    this->boh_taxels[40].vector.x = 0.007295;
    this->boh_taxels[40].vector.y = -0.020551;
    this->boh_taxels[40].vector.z = -0.0098;
    this->boh_taxels[41].vector.x = 0.015804;
    this->boh_taxels[41].vector.y = -0.020551;
    this->boh_taxels[41].vector.z = -0.0098;
    this->boh_taxels[42].vector.x = 0.024313;
    this->boh_taxels[42].vector.y = -0.020551;
    this->boh_taxels[42].vector.z = -0.0098;
    this->boh_taxels[43].vector.x = 0.032822;
    this->boh_taxels[43].vector.y = -0.020551;
    this->boh_taxels[43].vector.z = -0.0098;
    this->boh_taxels[44].vector.x = 0.041331;
    this->boh_taxels[44].vector.y = -0.020551;
    this->boh_taxels[44].vector.z = -0.0098;
    this->boh_taxels[45].vector.x = 0.04984;
    this->boh_taxels[45].vector.y = -0.020551;
    this->boh_taxels[45].vector.z = -0.0098;
    this->boh_taxels[46].vector.x = 0.058349;
    this->boh_taxels[46].vector.y = -0.020551;
    this->boh_taxels[46].vector.z = -0.0098;
    this->boh_taxels[47].vector.x = 0.066858;
    this->boh_taxels[47].vector.y = -0.020551;
    this->boh_taxels[47].vector.z = -0.0098;
    //Row5
    this->boh_taxels[48].vector.x = -0.026741;
    this->boh_taxels[48].vector.y = -0.027252;
    this->boh_taxels[48].vector.z = -0.0098;
    this->boh_taxels[49].vector.x = -0.018232;
    this->boh_taxels[49].vector.y = -0.027252;
    this->boh_taxels[49].vector.z = -0.0098;
    this->boh_taxels[50].vector.x = -0.009723;
    this->boh_taxels[50].vector.y = -0.027252;
    this->boh_taxels[50].vector.z = -0.0098;
    this->boh_taxels[51].vector.x = -0.001214;
    this->boh_taxels[51].vector.y = -0.027252;
    this->boh_taxels[51].vector.z = -0.0098;
    this->boh_taxels[52].vector.x = 0.007295;
    this->boh_taxels[52].vector.y = -0.027252;
    this->boh_taxels[52].vector.z = -0.0098;
    this->boh_taxels[53].vector.x = 0.015804;
    this->boh_taxels[53].vector.y = -0.027252;
    this->boh_taxels[53].vector.z = -0.0098;
    this->boh_taxels[54].vector.x = 0.024313;
    this->boh_taxels[54].vector.y = -0.027252;
    this->boh_taxels[54].vector.z = -0.0098;
    this->boh_taxels[55].vector.x = 0.032822;
    this->boh_taxels[55].vector.y = -0.027252;
    this->boh_taxels[55].vector.z = -0.0098;
    this->boh_taxels[56].vector.x = 0.041331;
    this->boh_taxels[56].vector.y = -0.027252;
    this->boh_taxels[56].vector.z = -0.0098;
    this->boh_taxels[57].vector.x = 0.04984;
    this->boh_taxels[57].vector.y = -0.027252;
    this->boh_taxels[57].vector.z = -0.0098;
    this->boh_taxels[58].vector.x = 0.058349;
    this->boh_taxels[58].vector.y = -0.027252;
    this->boh_taxels[58].vector.z = -0.0098;
    this->boh_taxels[59].vector.x = 0.066858;
    this->boh_taxels[59].vector.y = -0.027252;
    this->boh_taxels[59].vector.z = -0.0098;
    //Row6
    this->boh_taxels[60].vector.x = -0.026741;
    this->boh_taxels[60].vector.y = -0.033954;
    this->boh_taxels[60].vector.z = -0.0098;
    this->boh_taxels[61].vector.x = -0.018232;
    this->boh_taxels[61].vector.y = -0.033954;
    this->boh_taxels[61].vector.z = -0.0098;
    this->boh_taxels[62].vector.x = -0.009723;
    this->boh_taxels[62].vector.y = -0.033954;
    this->boh_taxels[62].vector.z = -0.0098;
    this->boh_taxels[63].vector.x = -0.001214;
    this->boh_taxels[63].vector.y = -0.033954;
    this->boh_taxels[63].vector.z = -0.0098;
    this->boh_taxels[64].vector.x = 0.007295;
    this->boh_taxels[64].vector.y = -0.033954;
    this->boh_taxels[64].vector.z = -0.0098;
    this->boh_taxels[65].vector.x = 0.015804;
    this->boh_taxels[65].vector.y = -0.033954;
    this->boh_taxels[65].vector.z = -0.0098;
    this->boh_taxels[66].vector.x = 0.024313;
    this->boh_taxels[66].vector.y = -0.033954;
    this->boh_taxels[66].vector.z = -0.0098;
    this->boh_taxels[67].vector.x = 0.032822;
    this->boh_taxels[67].vector.y = -0.033954;
    this->boh_taxels[67].vector.z = -0.0098;
    this->boh_taxels[68].vector.x = 0.041331;
    this->boh_taxels[68].vector.y = -0.033954;
    this->boh_taxels[68].vector.z = -0.0098;
    this->boh_taxels[69].vector.x = 0.04984;
    this->boh_taxels[69].vector.y = -0.033954;
    this->boh_taxels[69].vector.z = -0.0098;
    this->boh_taxels[70].vector.x = 0.058349;
    this->boh_taxels[70].vector.y = -0.033954;
    this->boh_taxels[70].vector.z = -0.0098;
    this->boh_taxels[71].vector.x = 0.066858;
    this->boh_taxels[71].vector.y = -0.033954;
    this->boh_taxels[71].vector.z = -0.0098;
    //Row7
    this->boh_taxels[72].vector.x = -0.026741;
    this->boh_taxels[72].vector.y = -0.040611;
    this->boh_taxels[72].vector.z = -0.0098;
    this->boh_taxels[73].vector.x = -0.018232;
    this->boh_taxels[73].vector.y = -0.040611;
    this->boh_taxels[73].vector.z = -0.0098;
    this->boh_taxels[74].vector.x = -0.009723;
    this->boh_taxels[74].vector.y = -0.040611;
    this->boh_taxels[74].vector.z = -0.0098;
    this->boh_taxels[75].vector.x = -0.001214;
    this->boh_taxels[75].vector.y = -0.040611;
    this->boh_taxels[75].vector.z = -0.0098;
    this->boh_taxels[76].vector.x = 0.007295;
    this->boh_taxels[76].vector.y = -0.040611;
    this->boh_taxels[76].vector.z = -0.0098;
    this->boh_taxels[77].vector.x = 0.015804;
    this->boh_taxels[77].vector.y = -0.040611;
    this->boh_taxels[77].vector.z = -0.0098;
    this->boh_taxels[78].vector.x = 0.024313;
    this->boh_taxels[78].vector.y = -0.040611;
    this->boh_taxels[78].vector.z = -0.0098;
    this->boh_taxels[79].vector.x = 0.032822;
    this->boh_taxels[79].vector.y = -0.040611;
    this->boh_taxels[79].vector.z = -0.0098;
    this->boh_taxels[80].vector.x = 0.041331;
    this->boh_taxels[80].vector.y = -0.040611;
    this->boh_taxels[80].vector.z = -0.0098;
    this->boh_taxels[81].vector.x = 0.04984;
    this->boh_taxels[81].vector.y = -0.040611;
    this->boh_taxels[81].vector.z = -0.0098;
    this->boh_taxels[82].vector.x = 0.058349;
    this->boh_taxels[82].vector.y = -0.040611;
    this->boh_taxels[82].vector.z = -0.0098;
    this->boh_taxels[83].vector.x = 0.066858;
    this->boh_taxels[83].vector.y = -0.040611;
    this->boh_taxels[83].vector.z = -0.0098;
    //Row8
    this->boh_taxels[84].vector.x = -0.026741;
    this->boh_taxels[84].vector.y = -0.047356;
    this->boh_taxels[84].vector.z = -0.0098;
    this->boh_taxels[85].vector.x = -0.018232;
    this->boh_taxels[85].vector.y = -0.047356;
    this->boh_taxels[85].vector.z = -0.0098;
    this->boh_taxels[86].vector.x = -0.009723;
    this->boh_taxels[86].vector.y = -0.047356;
    this->boh_taxels[86].vector.z = -0.0098;
    this->boh_taxels[87].vector.x = -0.001214;
    this->boh_taxels[87].vector.y = -0.047356;
    this->boh_taxels[87].vector.z = -0.0098;
    this->boh_taxels[88].vector.x = 0.007295;
    this->boh_taxels[88].vector.y = -0.047356;
    this->boh_taxels[88].vector.z = -0.0098;
    this->boh_taxels[89].vector.x = 0.015804;
    this->boh_taxels[89].vector.y = -0.047356;
    this->boh_taxels[89].vector.z = -0.0098;
    this->boh_taxels[90].vector.x = 0.024313;
    this->boh_taxels[90].vector.y = -0.047356;
    this->boh_taxels[90].vector.z = -0.0098;
    this->boh_taxels[91].vector.x = 0.032822;
    this->boh_taxels[91].vector.y = -0.047356;
    this->boh_taxels[91].vector.z = -0.0098;
    this->boh_taxels[92].vector.x = 0.041331;
    this->boh_taxels[92].vector.y = -0.047356;
    this->boh_taxels[92].vector.z = -0.0098;
    this->boh_taxels[93].vector.x = 0.04984;
    this->boh_taxels[93].vector.y = -0.047356;
    this->boh_taxels[93].vector.z = -0.0098;
    this->boh_taxels[94].vector.x = 0.058349;
    this->boh_taxels[94].vector.y = -0.047356;
    this->boh_taxels[94].vector.z = -0.0098;
    this->boh_taxels[95].vector.x = 0.066858;
    this->boh_taxels[95].vector.y = -0.047356;
    this->boh_taxels[95].vector.z = -0.0098;
    //Row9
    this->boh_taxels[96].vector.x = -0.026923;
    this->boh_taxels[96].vector.y = -0.054057;
    this->boh_taxels[96].vector.z = -0.0098;
    this->boh_taxels[97].vector.x = -0.018414;
    this->boh_taxels[97].vector.y = -0.054057;
    this->boh_taxels[97].vector.z = -0.0098;
    this->boh_taxels[98].vector.x = -0.009905;
    this->boh_taxels[98].vector.y = -0.054057;
    this->boh_taxels[98].vector.z = -0.0098;
    this->boh_taxels[99].vector.x = -0.001396;
    this->boh_taxels[99].vector.y = -0.054057;
    this->boh_taxels[99].vector.z = -0.0098;
    this->boh_taxels[100].vector.x = 0.007113;
    this->boh_taxels[100].vector.y = -0.054057;
    this->boh_taxels[100].vector.z = -0.0098;
    this->boh_taxels[101].vector.x = 0.015622;
    this->boh_taxels[101].vector.y = -0.054057;
    this->boh_taxels[101].vector.z = -0.0098;
    this->boh_taxels[102].vector.x = 0.024131;
    this->boh_taxels[102].vector.y = -0.054057;
    this->boh_taxels[102].vector.z = -0.0098;
    this->boh_taxels[103].vector.x = 0.03264;
    this->boh_taxels[103].vector.y = -0.054057;
    this->boh_taxels[103].vector.z = -0.0098;
    this->boh_taxels[104].vector.x = 0.041149;
    this->boh_taxels[104].vector.y = -0.054057;
    this->boh_taxels[104].vector.z = -0.0098;
    this->boh_taxels[105].vector.x = 0.049658;
    this->boh_taxels[105].vector.y = -0.054057;
    this->boh_taxels[105].vector.z = -0.0098;
    this->boh_taxels[106].vector.x = 0.058167;
    this->boh_taxels[106].vector.y = -0.054057;
    this->boh_taxels[106].vector.z = -0.0098;
    this->boh_taxels[107].vector.x = 0.066676;
    this->boh_taxels[107].vector.y = -0.054057;
    this->boh_taxels[107].vector.z = -0.0098;
    //Row10
    this->boh_taxels[108].vector.x = -0.026928;
    this->boh_taxels[108].vector.y = -0.060963;
    this->boh_taxels[108].vector.z = -0.0098;
    this->boh_taxels[109].vector.x = -0.018419;
    this->boh_taxels[109].vector.y = -0.060963;
    this->boh_taxels[109].vector.z = -0.0098;
    this->boh_taxels[110].vector.x = -0.00991;
    this->boh_taxels[110].vector.y = -0.060963;
    this->boh_taxels[110].vector.z = -0.0098;
    this->boh_taxels[111].vector.x = 0.049658;
    this->boh_taxels[111].vector.y = -0.060899;
    this->boh_taxels[111].vector.z = -0.0098;
    this->boh_taxels[112].vector.x = 0.058167;
    this->boh_taxels[112].vector.y = -0.060899;
    this->boh_taxels[112].vector.z = -0.0098;
    this->boh_taxels[113].vector.x = 0.066731;
    this->boh_taxels[113].vector.y = -0.060899;
    this->boh_taxels[113].vector.z = -0.0098;
    //Row11
    this->boh_taxels[114].vector.x = -0.018419;
    this->boh_taxels[114].vector.y = -0.068096;
    this->boh_taxels[114].vector.z = -0.0098;
    this->boh_taxels[115].vector.x = -0.00991;
    this->boh_taxels[115].vector.y = -0.068096;
    this->boh_taxels[115].vector.z = -0.0098;
    this->boh_taxels[116].vector.x = 0.049658;
    this->boh_taxels[116].vector.y = -0.067905;
    this->boh_taxels[116].vector.z = -0.0098;
    this->boh_taxels[117].vector.x = 0.058167;
    this->boh_taxels[117].vector.y = -0.067905;
    this->boh_taxels[117].vector.z = -0.0098;

    //PALM

    //UPPER MATRIX
    //Row1
    this->palm_taxels[0].vector.x = -0.04382;
    this->palm_taxels[0].vector.y = -0.083961;
    this->palm_taxels[0].vector.z = -0.0061;
    this->palm_taxels[1].vector.x = -0.04382;
    this->palm_taxels[1].vector.y = -0.079524;
    this->palm_taxels[1].vector.z = -0.0061;
    this->palm_taxels[2].vector.x = -0.04382;
    this->palm_taxels[2].vector.y = -0.075087;
    this->palm_taxels[2].vector.z = -0.0061;
    this->palm_taxels[3].vector.x = -0.04382;
    this->palm_taxels[3].vector.y =-0.07065;
    this->palm_taxels[3].vector.z = -0.0061;
    this->palm_taxels[4].vector.x = -0.04382;
    this->palm_taxels[4].vector.y = -0.066213;
    this->palm_taxels[4].vector.z = -0.0061;
    this->palm_taxels[5].vector.x = -0.04382;
    this->palm_taxels[5].vector.y = -0.061776;
    this->palm_taxels[5].vector.z = -0.0061;
    //Row2
    this->palm_taxels[6].vector.x = -0.036508;
    this->palm_taxels[6].vector.y = -0.083961;
    this->palm_taxels[6].vector.z = -0.0061;
    this->palm_taxels[7].vector.x = -0.036508;
    this->palm_taxels[7].vector.y = -0.079524;
    this->palm_taxels[7].vector.z = -0.0061;
    this->palm_taxels[8].vector.x = -0.036508;
    this->palm_taxels[8].vector.y = -0.075087;
    this->palm_taxels[8].vector.z = -0.0061;
    this->palm_taxels[9].vector.x = -0.036508;
    this->palm_taxels[9].vector.y = -0.07065;
    this->palm_taxels[9].vector.z = -0.0061;
    this->palm_taxels[10].vector.x = -0.036508;
    this->palm_taxels[10].vector.y = -0.066213;
    this->palm_taxels[10].vector.z = -0.0061;
    this->palm_taxels[11].vector.x = -0.036508;
    this->palm_taxels[11].vector.y = -0.061776;
    this->palm_taxels[11].vector.z = -0.0061;
    //Row3
    this->palm_taxels[12].vector.x = -0.029196;
    this->palm_taxels[12].vector.y = -0.083961;
    this->palm_taxels[12].vector.z = -0.0061;
    this->palm_taxels[13].vector.x = -0.029196;
    this->palm_taxels[13].vector.y = -0.079524;
    this->palm_taxels[13].vector.z = -0.0061;
    this->palm_taxels[14].vector.x = -0.029196;
    this->palm_taxels[14].vector.y = -0.075087;
    this->palm_taxels[14].vector.z = -0.0061;
    this->palm_taxels[15].vector.x = -0.029196;
    this->palm_taxels[15].vector.y = -0.07065;
    this->palm_taxels[15].vector.z = -0.0061;
    this->palm_taxels[16].vector.x = -0.029196;
    this->palm_taxels[16].vector.y = -0.066213;
    this->palm_taxels[16].vector.z = -0.0061;
    this->palm_taxels[17].vector.x = -0.029196;
    this->palm_taxels[17].vector.y = -0.061776;
    this->palm_taxels[17].vector.z = -0.0061;
    //Row4
    this->palm_taxels[18].vector.x = -0.021884;
    this->palm_taxels[18].vector.y = -0.083961;
    this->palm_taxels[18].vector.z = -0.0061;
    this->palm_taxels[19].vector.x = -0.021884;
    this->palm_taxels[19].vector.y = -0.079524;
    this->palm_taxels[19].vector.z = -0.0061;
    this->palm_taxels[20].vector.x = -0.021884;
    this->palm_taxels[20].vector.y = -0.075087;
    this->palm_taxels[20].vector.z = -0.0061;
    this->palm_taxels[21].vector.x = -0.021884;
    this->palm_taxels[21].vector.y = -0.07065;
    this->palm_taxels[21].vector.z = -0.0061;
    this->palm_taxels[22].vector.x = -0.021884;
    this->palm_taxels[22].vector.y = -0.066213;
    this->palm_taxels[22].vector.z = -0.0061;
    this->palm_taxels[23].vector.x = -0.021884;
    this->palm_taxels[23].vector.y = -0.061776;
    this->palm_taxels[23].vector.z = -0.0061;
    //Row5
    //Row4
    this->palm_taxels[24].vector.x = -0.014572;
    this->palm_taxels[24].vector.y = -0.083961;
    this->palm_taxels[24].vector.z = -0.0061;
    this->palm_taxels[25].vector.x = -0.014572;
    this->palm_taxels[25].vector.y = -0.079524;
    this->palm_taxels[25].vector.z = -0.0061;
    this->palm_taxels[26].vector.x = -0.014572;
    this->palm_taxels[26].vector.y = -0.075087;
    this->palm_taxels[26].vector.z = -0.0061;
    this->palm_taxels[27].vector.x = -0.014572;
    this->palm_taxels[27].vector.y = -0.07065;
    this->palm_taxels[27].vector.z = -0.0061;
    this->palm_taxels[28].vector.x = -0.014572;
    this->palm_taxels[28].vector.y = -0.066213;
    this->palm_taxels[28].vector.z = -0.0061;
    this->palm_taxels[29].vector.x = -0.014572;
    this->palm_taxels[29].vector.y = -0.061776;
    this->palm_taxels[29].vector.z = -0.0061;
    //Row6
    this->palm_taxels[30].vector.x = -0.007259;
    this->palm_taxels[30].vector.y = -0.083961;
    this->palm_taxels[30].vector.z = -0.0061;
    this->palm_taxels[31].vector.x = -0.007259;
    this->palm_taxels[31].vector.y = -0.079524;
    this->palm_taxels[31].vector.z = -0.0061;
    this->palm_taxels[32].vector.x = -0.007259;
    this->palm_taxels[32].vector.y = -0.075087;
    this->palm_taxels[32].vector.z = -0.0061;
    this->palm_taxels[33].vector.x = -0.007259;
    this->palm_taxels[33].vector.y = -0.07065;
    this->palm_taxels[33].vector.z = -0.0061;
    this->palm_taxels[34].vector.x = -0.007259;
    this->palm_taxels[34].vector.y = -0.066213;
    this->palm_taxels[34].vector.z = -0.0061;
    this->palm_taxels[35].vector.x = -0.007259;
    this->palm_taxels[35].vector.y = -0.061776;
    this->palm_taxels[35].vector.z = -0.0061;
    //Row7
    this->palm_taxels[36].vector.x = 0.000153;
    this->palm_taxels[36].vector.y = -0.083961;
    this->palm_taxels[36].vector.z = -0.0061;
    this->palm_taxels[37].vector.x = 0.000153;
    this->palm_taxels[37].vector.y = -0.079524;
    this->palm_taxels[37].vector.z = -0.0061;
    this->palm_taxels[38].vector.x = 0.000153;
    this->palm_taxels[38].vector.y = -0.075087;
    this->palm_taxels[38].vector.z = -0.0061;
    this->palm_taxels[39].vector.x = 0.000153;
    this->palm_taxels[39].vector.y = -0.07065;
    this->palm_taxels[39].vector.z = -0.0061;
    this->palm_taxels[40].vector.x = 0.000153;
    this->palm_taxels[40].vector.y = -0.066213;
    this->palm_taxels[40].vector.z = -0.0061;
    this->palm_taxels[41].vector.x = 0.000153;
    this->palm_taxels[41].vector.y = -0.061776;
    this->palm_taxels[41].vector.z = -0.0061;
    //Row8
    this->palm_taxels[42].vector.x = 0.007465;
    this->palm_taxels[42].vector.y = -0.083961;
    this->palm_taxels[42].vector.z = -0.0061;
    this->palm_taxels[43].vector.x = 0.007465;
    this->palm_taxels[43].vector.y = -0.079524;
    this->palm_taxels[43].vector.z = -0.0061;
    this->palm_taxels[44].vector.x = 0.007465;
    this->palm_taxels[44].vector.y = -0.075087;
    this->palm_taxels[44].vector.z = -0.0061;
    this->palm_taxels[45].vector.x = 0.007465;
    this->palm_taxels[45].vector.y = -0.07065;
    this->palm_taxels[45].vector.z = -0.0061;
    this->palm_taxels[46].vector.x = 0.007465;
    this->palm_taxels[46].vector.y = -0.066213;
    this->palm_taxels[46].vector.z = -0.0061;
    this->palm_taxels[47].vector.x = 0.007465;
    this->palm_taxels[47].vector.y = -0.061776;
    this->palm_taxels[47].vector.z = -0.0061;
    //Row9
    this->palm_taxels[48].vector.x = 0.014777;
    this->palm_taxels[48].vector.y = -0.083961;
    this->palm_taxels[48].vector.z = -0.0061;
    this->palm_taxels[49].vector.x = 0.014777;
    this->palm_taxels[49].vector.y = -0.079524;
    this->palm_taxels[49].vector.z = -0.0061;
    this->palm_taxels[50].vector.x = 0.014777;
    this->palm_taxels[50].vector.y = -0.075087;
    this->palm_taxels[50].vector.z = -0.0061;
    this->palm_taxels[51].vector.x = 0.014777;
    this->palm_taxels[51].vector.y = -0.07065;
    this->palm_taxels[51].vector.z = -0.0061;
    this->palm_taxels[52].vector.x = 0.014777;
    this->palm_taxels[52].vector.y = -0.066213;
    this->palm_taxels[52].vector.z = -0.0061;
    this->palm_taxels[53].vector.x = 0.014777;
    this->palm_taxels[53].vector.y = -0.061776;
    this->palm_taxels[53].vector.z = -0.0061;
    //Row10
    this->palm_taxels[54].vector.x = 0.022089;
    this->palm_taxels[54].vector.y = -0.083961;
    this->palm_taxels[54].vector.z = -0.0061;
    this->palm_taxels[55].vector.x = 0.022089;
    this->palm_taxels[55].vector.y = -0.079524;
    this->palm_taxels[55].vector.z = -0.0061;
    this->palm_taxels[56].vector.x = 0.022089;
    this->palm_taxels[56].vector.y = -0.075087;
    this->palm_taxels[56].vector.z = -0.0061;
    this->palm_taxels[57].vector.x = 0.022089;
    this->palm_taxels[57].vector.y = -0.07065;
    this->palm_taxels[57].vector.z = -0.0061;
    this->palm_taxels[58].vector.x = 0.022089;
    this->palm_taxels[58].vector.y = -0.066213;
    this->palm_taxels[58].vector.z = -0.0061;
    this->palm_taxels[59].vector.x = 0.022089;
    this->palm_taxels[59].vector.y = -0.061776;
    this->palm_taxels[59].vector.z = -0.0061;
    //Row11
    this->palm_taxels[60].vector.x = 0.029401;
    this->palm_taxels[60].vector.y = -0.083961;
    this->palm_taxels[60].vector.z = -0.0061;
    this->palm_taxels[61].vector.x = 0.029401;
    this->palm_taxels[61].vector.y = -0.079524;
    this->palm_taxels[61].vector.z = -0.0061;
    this->palm_taxels[62].vector.x = 0.029401;
    this->palm_taxels[62].vector.y = -0.075087;
    this->palm_taxels[62].vector.z = -0.0061;
    this->palm_taxels[63].vector.x = 0.029401;
    this->palm_taxels[63].vector.y = -0.07065;
    this->palm_taxels[63].vector.z = -0.0061;
    this->palm_taxels[64].vector.x = 0.029401;
    this->palm_taxels[64].vector.y = -0.066213;
    this->palm_taxels[64].vector.z = -0.0061;
    this->palm_taxels[65].vector.x = 0.029401;
    this->palm_taxels[65].vector.y = -0.061776;
    this->palm_taxels[65].vector.z = -0.0061;
    //LOWER MATRIX
    //Row1
    this->palm_taxels[66].vector.x = -0.044614;
    this->palm_taxels[66].vector.y = -0.053013;
    this->palm_taxels[66].vector.z = -0.0061;
    this->palm_taxels[67].vector.x = -0.044233;
    this->palm_taxels[67].vector.y = -0.04774;
    this->palm_taxels[67].vector.z = -0.0061;
    this->palm_taxels[68].vector.x = -0.043878;
    this->palm_taxels[68].vector.y = -0.042467;
    this->palm_taxels[68].vector.z = -0.0061;
    this->palm_taxels[69].vector.x = -0.04339;
    this->palm_taxels[69].vector.y = -0.037194;
    this->palm_taxels[69].vector.z = -0.0061;
    this->palm_taxels[70].vector.x = -0.042969;
    this->palm_taxels[70].vector.y = -0.031921;
    this->palm_taxels[70].vector.z = -0.0061;
    this->palm_taxels[71].vector.x = -0.042455;
    this->palm_taxels[71].vector.y = -0.026648;
    this->palm_taxels[71].vector.z = -0.0061;
    //Row2
    this->palm_taxels[72].vector.x = -0.038391;
    this->palm_taxels[72].vector.y = -0.053013;
    this->palm_taxels[72].vector.z = -0.0061;
    this->palm_taxels[73].vector.x = -0.038137;
    this->palm_taxels[73].vector.y = -0.04774;
    this->palm_taxels[73].vector.z = -0.0061;
    this->palm_taxels[74].vector.x = -0.037909;
    this->palm_taxels[74].vector.y = -0.042467;
    this->palm_taxels[74].vector.z = -0.0061;
    this->palm_taxels[75].vector.x = -0.037548;
    this->palm_taxels[75].vector.y = -0.037194;
    this->palm_taxels[75].vector.z = -0.0061;
    this->palm_taxels[76].vector.x = -0.037254;
    this->palm_taxels[76].vector.y = -0.031921;
    this->palm_taxels[76].vector.z = -0.0061;
    this->palm_taxels[77].vector.x = -0.036867;
    this->palm_taxels[77].vector.y = -0.026648;
    this->palm_taxels[77].vector.z = -0.0061;
    //Row3
    this->palm_taxels[78].vector.x = -0.032168;
    this->palm_taxels[78].vector.y = -0.053013;
    this->palm_taxels[78].vector.z = -0.0061;
    this->palm_taxels[79].vector.x = -0.032041;
    this->palm_taxels[79].vector.y = -0.04774;
    this->palm_taxels[79].vector.z = -0.0061;
    this->palm_taxels[80].vector.x = -0.03194;
    this->palm_taxels[80].vector.y = -0.042467;
    this->palm_taxels[80].vector.z = -0.0061;
    this->palm_taxels[81].vector.x = -0.031706;
    this->palm_taxels[81].vector.y = -0.037194;
    this->palm_taxels[81].vector.z = -0.0061;
    this->palm_taxels[82].vector.x = -0.031539;
    this->palm_taxels[82].vector.y = -0.031921;
    this->palm_taxels[82].vector.z = -0.0061;
    this->palm_taxels[83].vector.x = -0.031279;
    this->palm_taxels[83].vector.y = -0.026648;
    this->palm_taxels[83].vector.z = -0.0061;
    //Row4
    this->palm_taxels[84].vector.x = -0.025945;
    this->palm_taxels[84].vector.y = -0.053013;
    this->palm_taxels[84].vector.z = -0.0061;
    this->palm_taxels[85].vector.x = -0.025945;
    this->palm_taxels[85].vector.y = -0.04774;
    this->palm_taxels[85].vector.z = -0.0061;
    this->palm_taxels[86].vector.x = -0.025971;
    this->palm_taxels[86].vector.y = -0.042467;
    this->palm_taxels[86].vector.z = -0.0061;
    this->palm_taxels[87].vector.x = -0.025864;
    this->palm_taxels[87].vector.y = -0.037194;
    this->palm_taxels[87].vector.z = -0.0061;
    this->palm_taxels[88].vector.x = -0.025824;
    this->palm_taxels[88].vector.y = -0.031921;
    this->palm_taxels[88].vector.z = -0.0061;
    this->palm_taxels[89].vector.x = -0.025691;
    this->palm_taxels[89].vector.y = -0.026648;
    this->palm_taxels[89].vector.z = -0.0061;
    //Row5
    this->palm_taxels[90].vector.x = -0.019722;
    this->palm_taxels[90].vector.y = -0.053013;
    this->palm_taxels[90].vector.z = -0.0061;
    this->palm_taxels[91].vector.x = -0.019849;
    this->palm_taxels[91].vector.y = -0.04774;
    this->palm_taxels[91].vector.z = -0.0061;
    this->palm_taxels[92].vector.x = -0.020002;
    this->palm_taxels[92].vector.y = -0.042467;
    this->palm_taxels[92].vector.z = -0.0061;
    this->palm_taxels[93].vector.x = -0.020002;
    this->palm_taxels[93].vector.y = -0.037194;
    this->palm_taxels[93].vector.z = -0.0061;
    this->palm_taxels[94].vector.x = -0.020109;
    this->palm_taxels[94].vector.y = -0.031921;
    this->palm_taxels[94].vector.z = -0.0061;
    this->palm_taxels[95].vector.x = -0.020103;
    this->palm_taxels[95].vector.y = -0.026648;
    this->palm_taxels[95].vector.z = -0.0061;
    //Row6
    this->palm_taxels[96].vector.x = -0.013499;
    this->palm_taxels[96].vector.y = -0.053013;
    this->palm_taxels[96].vector.z = -0.0061;
    this->palm_taxels[97].vector.x = -0.013753;
    this->palm_taxels[97].vector.y = -0.04774;
    this->palm_taxels[97].vector.z = -0.0061;
    this->palm_taxels[98].vector.x = -0.014033;
    this->palm_taxels[98].vector.y = -0.042467;
    this->palm_taxels[98].vector.z = -0.0061;
    this->palm_taxels[99].vector.x = -0.014180;
    this->palm_taxels[99].vector.y = -0.037194;
    this->palm_taxels[99].vector.z = -0.0061;
    this->palm_taxels[100].vector.x = -0.020109;
    this->palm_taxels[100].vector.y = -0.031921;
    this->palm_taxels[100].vector.z = -0.0061;
    this->palm_taxels[101].vector.x = -0.014394;
    this->palm_taxels[101].vector.y = -0.026648;
    this->palm_taxels[101].vector.z = -0.0061;
    //Row7
    this->palm_taxels[102].vector.x = -0.007221;
    this->palm_taxels[102].vector.y = -0.053013;
    this->palm_taxels[102].vector.z = -0.0061;
    this->palm_taxels[103].vector.x = -0.007657;
    this->palm_taxels[103].vector.y = -0.04774;
    this->palm_taxels[103].vector.z = -0.0061;
    this->palm_taxels[104].vector.x = -0.008064;
    this->palm_taxels[104].vector.y = -0.042467;
    this->palm_taxels[104].vector.z = -0.0061;
    this->palm_taxels[105].vector.x = -0.008338;
    this->palm_taxels[105].vector.y = -0.037194;
    this->palm_taxels[105].vector.z = -0.0061;
    this->palm_taxels[106].vector.x = -0.008679;
    this->palm_taxels[106].vector.y = -0.031921;
    this->palm_taxels[106].vector.z = -0.0061;
    this->palm_taxels[107].vector.x = -0.008927;
    this->palm_taxels[107].vector.y = -0.026648;
    this->palm_taxels[107].vector.z = -0.0061;
    //UPPER SMALL MATRIX
    this->palm_taxels[108].vector.x = 0.04043;
    this->palm_taxels[108].vector.y = -0.085037;
    this->palm_taxels[108].vector.z = -0.0061;
    this->palm_taxels[109].vector.x = 0.04043;
    this->palm_taxels[109].vector.y = -0.080719;
    this->palm_taxels[109].vector.z = -0.0061;
    this->palm_taxels[110].vector.x = 0.045611;
    this->palm_taxels[110].vector.y = -0.085037;
    this->palm_taxels[110].vector.z = -0.0061;
    this->palm_taxels[111].vector.x = 0.045611;
    this->palm_taxels[111].vector.y = -0.080719;
    this->palm_taxels[111].vector.z = -0.0061;
    //LOWER SMALL MATRIX
    this->palm_taxels[112].vector.x = 0.04043;
    this->palm_taxels[112].vector.y = -0.06359;
    this->palm_taxels[112].vector.z = -0.0061;
    this->palm_taxels[113].vector.x = 0.04043;
    this->palm_taxels[113].vector.y = -0.059201;
    this->palm_taxels[113].vector.z = -0.0061;
    this->palm_taxels[114].vector.x = 0.045611;
    this->palm_taxels[114].vector.y = -0.06359;
    this->palm_taxels[114].vector.z = -0.0061;
    this->palm_taxels[115].vector.x = 0.045611;
    this->palm_taxels[115].vector.y = -0.059201;
    this->palm_taxels[115].vector.z = -0.0061;
    //LOWER MATRIX BOTTOM LINE
    this->palm_taxels[116].vector.x = -0.037374;
    this->palm_taxels[116].vector.y = -0.021375;
    this->palm_taxels[116].vector.z = -0.0061;
    this->palm_taxels[117].vector.x = -0.031459;
    this->palm_taxels[117].vector.y = -0.021375;
    this->palm_taxels[117].vector.z = -0.0061;
    this->palm_taxels[118].vector.x = -0.025544;
    this->palm_taxels[118].vector.y = -0.021375;
    this->palm_taxels[118].vector.z = -0.0061;
    this->palm_taxels[119].vector.x = -0.019629;
    this->palm_taxels[119].vector.y = -0.021375;
    this->palm_taxels[119].vector.z = -0.0061;
    this->palm_taxels[120].vector.x = -0.013713;
    this->palm_taxels[120].vector.y = -0.021375;
    this->palm_taxels[120].vector.z = -0.0061;






    //Index Proximal Front

    //Row1
    this->ipf_taxels[0].vector.x = -0.024275;
    this->ipf_taxels[0].vector.y = 0.008283;
    this->ipf_taxels[0].vector.z = 0.012385;
    this->ipf_taxels[1].vector.x = -0.01934;
    this->ipf_taxels[1].vector.y = 0.008283;
    this->ipf_taxels[1].vector.z = 0.011953;
    this->ipf_taxels[2].vector.x = -0.014452;
    this->ipf_taxels[2].vector.y = 0.008283;
    this->ipf_taxels[2].vector.z = 0.011525;
    this->ipf_taxels[3].vector.x = -0.00773;
    this->ipf_taxels[3].vector.y = 0.008283;
    this->ipf_taxels[3].vector.z = 0.010937;
    this->ipf_taxels[4].vector.x = -0.002418;
    this->ipf_taxels[4].vector.y = 0.008283;
    this->ipf_taxels[4].vector.z = 0.010521;
    this->ipf_taxels[5].vector.x = 0.002881;
    this->ipf_taxels[5].vector.y = 0.008283;
    this->ipf_taxels[5].vector.z = 0.010008;
    this->ipf_taxels[6].vector.x = 0.008014;
    this->ipf_taxels[6].vector.y = 0.008283;
    this->ipf_taxels[6].vector.z = 0.009564;
    this->ipf_taxels[7].vector.x = 0.013002;
    this->ipf_taxels[7].vector.y = 0.008283;
    this->ipf_taxels[7].vector.z = 0.009122;
    this->ipf_taxels[8].vector.x = 0.019755;
    this->ipf_taxels[8].vector.y = 0.008283;
    this->ipf_taxels[8].vector.z = 0.008553;
    this->ipf_taxels[9].vector.x = 0.024652;
    this->ipf_taxels[9].vector.y = 0.008283;
    this->ipf_taxels[9].vector.z = 0.008093;
    this->ipf_taxels[10].vector.x = 0.029593;
    this->ipf_taxels[10].vector.y = 0.008283;
    this->ipf_taxels[10].vector.z = 0.007671;
    //Row2
    this->ipf_taxels[11].vector.x = -0.02393;
    this->ipf_taxels[11].vector.y = 0.008418;
    this->ipf_taxels[11].vector.z = 0.016317;
    this->ipf_taxels[12].vector.x = -0.018996;
    this->ipf_taxels[12].vector.y = 0.008418;
    this->ipf_taxels[12].vector.z = 0.015885;
    this->ipf_taxels[13].vector.x = -0.014108;
    this->ipf_taxels[13].vector.y = 0.008418;
    this->ipf_taxels[13].vector.z = 0.015457;
    this->ipf_taxels[14].vector.x = -0.007386;
    this->ipf_taxels[14].vector.y = 0.008418;
    this->ipf_taxels[14].vector.z = 0.014869;
    this->ipf_taxels[15].vector.x = -0.002074;
    this->ipf_taxels[15].vector.y = 0.008418;
    this->ipf_taxels[15].vector.z = 0.014454;
    this->ipf_taxels[16].vector.x = 0.003225;
    this->ipf_taxels[16].vector.y = 0.008418;
    this->ipf_taxels[16].vector.z = 0.013941;
    this->ipf_taxels[17].vector.x = 0.008358;
    this->ipf_taxels[17].vector.y = 0.008418;
    this->ipf_taxels[17].vector.z = 0.013497;
    this->ipf_taxels[18].vector.x = 0.013346;
    this->ipf_taxels[18].vector.y = 0.008418;
    this->ipf_taxels[18].vector.z = 0.013055;
    this->ipf_taxels[19].vector.x = 0.020099;
    this->ipf_taxels[19].vector.y = 0.008418;
    this->ipf_taxels[19].vector.z = 0.012486;
    this->ipf_taxels[20].vector.x = 0.025033;
    this->ipf_taxels[20].vector.y = 0.008418;
    this->ipf_taxels[20].vector.z = 0.012054;
    this->ipf_taxels[21].vector.x = 0.029937;
    this->ipf_taxels[21].vector.y = 0.008418;
    this->ipf_taxels[21].vector.z = 0.011604;
    //Row3
    this->ipf_taxels[22].vector.x = -0.023585;
    this->ipf_taxels[22].vector.y = 0.008329;
    this->ipf_taxels[22].vector.z = 0.02027;
    this->ipf_taxels[23].vector.x = -0.01865;
    this->ipf_taxels[23].vector.y = 0.008329;
    this->ipf_taxels[23].vector.z = 0.019838;
    this->ipf_taxels[24].vector.x = -0.013762;
    this->ipf_taxels[24].vector.y = 0.008329;
    this->ipf_taxels[24].vector.z = 0.019411;
    this->ipf_taxels[25].vector.x = -0.00704;
    this->ipf_taxels[25].vector.y = 0.008329;
    this->ipf_taxels[25].vector.z = 0.018822;
    this->ipf_taxels[26].vector.x = -0.001728;
    this->ipf_taxels[26].vector.y = 0.008329;
    this->ipf_taxels[26].vector.z = 0.018407;
    this->ipf_taxels[27].vector.x = 0.003571;
    this->ipf_taxels[27].vector.y = 0.008329;
    this->ipf_taxels[27].vector.z = 0.017894;
    this->ipf_taxels[28].vector.x = 0.008703;
    this->ipf_taxels[28].vector.y = 0.008329;
    this->ipf_taxels[28].vector.z = 0.01745;
    this->ipf_taxels[29].vector.x = 0.013692;
    this->ipf_taxels[29].vector.y = 0.008329;
    this->ipf_taxels[29].vector.z = 0.017009;
    this->ipf_taxels[30].vector.x = 0.020443;
    this->ipf_taxels[30].vector.y = 0.008329;
    this->ipf_taxels[30].vector.z = 0.016418;
    this->ipf_taxels[31].vector.x = 0.02534;
    this->ipf_taxels[31].vector.y = 0.008329;
    this->ipf_taxels[31].vector.z = 0.015957;
    this->ipf_taxels[32].vector.x = 0.030283;
    this->ipf_taxels[32].vector.y = 0.008329;
    this->ipf_taxels[32].vector.z = 0.015557;
    //Row4
    this->ipf_taxels[33].vector.x = -0.009048;
    this->ipf_taxels[33].vector.y = 0.007277;
    this->ipf_taxels[33].vector.z = 0.024677;
    this->ipf_taxels[34].vector.x = -0.00409;
    this->ipf_taxels[34].vector.y = 0.007277;
    this->ipf_taxels[34].vector.z = 0.024221;
    this->ipf_taxels[35].vector.x = 0.001613;
    this->ipf_taxels[35].vector.y = 0.007277;
    this->ipf_taxels[35].vector.z = 0.02383;
    this->ipf_taxels[36].vector.x = 0.0068;
    this->ipf_taxels[36].vector.y = 0.007277;
    this->ipf_taxels[36].vector.z = 0.023376;
    this->ipf_taxels[37].vector.x = 0.012334;
    this->ipf_taxels[37].vector.y = 0.007277;
    this->ipf_taxels[37].vector.z = 0.022812;
    this->ipf_taxels[38].vector.x = 0.017016;
    this->ipf_taxels[38].vector.y = 0.007277;
    this->ipf_taxels[38].vector.z = 0.022411;
    //Row5
    this->ipf_taxels[39].vector.x = -0.008665;
    this->ipf_taxels[39].vector.y = 0.005852;
    this->ipf_taxels[39].vector.z = 0.029059;
    this->ipf_taxels[40].vector.x = -0.003707;
    this->ipf_taxels[40].vector.y = 0.005852;
    this->ipf_taxels[40].vector.z = 0.028605;
    this->ipf_taxels[41].vector.x = 0.001996;
    this->ipf_taxels[41].vector.y = 0.005852;
    this->ipf_taxels[41].vector.z = 0.028207;
    this->ipf_taxels[42].vector.x = 0.007183;
    this->ipf_taxels[42].vector.y = 0.005852;
    this->ipf_taxels[42].vector.z = 0.027754;
    this->ipf_taxels[43].vector.x = 0.012715;
    this->ipf_taxels[43].vector.y = 0.005852;
    this->ipf_taxels[43].vector.z = 0.027174;
    this->ipf_taxels[44].vector.x = 0.017398;
    this->ipf_taxels[44].vector.y = 0.005852;
    this->ipf_taxels[44].vector.z = 0.026779;
    //Row6
    this->ipf_taxels[45].vector.x = -0.008457;
    this->ipf_taxels[45].vector.y = 0.001986;
    this->ipf_taxels[45].vector.z = 0.031433;
    this->ipf_taxels[46].vector.x = -0.003497;
    this->ipf_taxels[46].vector.y = 0.001986;
    this->ipf_taxels[46].vector.z = 0.030997;
    this->ipf_taxels[47].vector.x = 0.012924;
    this->ipf_taxels[47].vector.y = 0.001986;
    this->ipf_taxels[47].vector.z = 0.029562;
    this->ipf_taxels[48].vector.x = 0.017605;
    this->ipf_taxels[48].vector.y = 0.001986;
    this->ipf_taxels[48].vector.z = 0.029153;
    //Row7
    this->ipf_taxels[49].vector.x = -0.008405;
    this->ipf_taxels[49].vector.y = -0.002328;
    this->ipf_taxels[49].vector.z = 0.03203;
    this->ipf_taxels[50].vector.x = -0.003445;
    this->ipf_taxels[50].vector.y = -0.002328;
    this->ipf_taxels[50].vector.z = 0.031595;
    this->ipf_taxels[51].vector.x = 0.012976;
    this->ipf_taxels[51].vector.y = -0.002328;
    this->ipf_taxels[51].vector.z = 0.030159;
    this->ipf_taxels[52].vector.x = 0.017658;
    this->ipf_taxels[52].vector.y = -0.002328;
    this->ipf_taxels[52].vector.z = 0.029749;
    //Row8
    this->ipf_taxels[53].vector.x = -0.008396;
    this->ipf_taxels[53].vector.y = -0.006292;
    this->ipf_taxels[53].vector.z = 0.032132;
    this->ipf_taxels[54].vector.x = -0.003436;
    this->ipf_taxels[54].vector.y = -0.006292;
    this->ipf_taxels[54].vector.z = 0.031698;
    this->ipf_taxels[55].vector.x = 0.012987;
    this->ipf_taxels[55].vector.y = -0.006292;
    this->ipf_taxels[55].vector.z = 0.030278;
    this->ipf_taxels[56].vector.x = 0.017668;
    this->ipf_taxels[56].vector.y = -0.006292;
    this->ipf_taxels[56].vector.z = 0.029869;
    //Row9
    this->ipf_taxels[57].vector.x = -0.008394;
    this->ipf_taxels[57].vector.y = -0.010743;
    this->ipf_taxels[57].vector.z = 0.032153;
    this->ipf_taxels[58].vector.x = -0.003434;
    this->ipf_taxels[58].vector.y = -0.010743;
    this->ipf_taxels[58].vector.z = 0.031719;
    this->ipf_taxels[59].vector.x = 0.012987;
    this->ipf_taxels[59].vector.y = -0.010743;
    this->ipf_taxels[59].vector.z = 0.030282;
    this->ipf_taxels[60].vector.x = 0.017668;
    this->ipf_taxels[60].vector.y = -0.010743;
    this->ipf_taxels[60].vector.z = 0.029873;
    //Row10
    this->ipf_taxels[61].vector.x = -0.008394;
    this->ipf_taxels[61].vector.y = -0.015192;
    this->ipf_taxels[61].vector.z = 0.032153;
    this->ipf_taxels[62].vector.x = -0.003434;
    this->ipf_taxels[62].vector.y = -0.015192;
    this->ipf_taxels[62].vector.z = 0.031719;
    this->ipf_taxels[63].vector.x = 0.012987;
    this->ipf_taxels[63].vector.y = -0.015192;
    this->ipf_taxels[63].vector.z = 0.030282;
    this->ipf_taxels[64].vector.x = 0.017668;
    this->ipf_taxels[64].vector.y = -0.015192;
    this->ipf_taxels[64].vector.z = 0.029873;

    //Middle Proximal Front

    //Row1
    this->mpf_taxels[0].vector.x = -0.024275;
    this->mpf_taxels[0].vector.y = 0.008283;
    this->mpf_taxels[0].vector.z = 0.012385;
    this->mpf_taxels[1].vector.x = -0.01934;
    this->mpf_taxels[1].vector.y = 0.008283;
    this->mpf_taxels[1].vector.z = 0.011953;
    this->mpf_taxels[2].vector.x = -0.014452;
    this->mpf_taxels[2].vector.y = 0.008283;
    this->mpf_taxels[2].vector.z = 0.011525;
    this->mpf_taxels[3].vector.x = -0.00773;
    this->mpf_taxels[3].vector.y = 0.008283;
    this->mpf_taxels[3].vector.z = 0.010937;
    this->mpf_taxels[4].vector.x = -0.002418;
    this->mpf_taxels[4].vector.y = 0.008283;
    this->mpf_taxels[4].vector.z = 0.010521;
    this->mpf_taxels[5].vector.x = 0.002881;
    this->mpf_taxels[5].vector.y = 0.008283;
    this->mpf_taxels[5].vector.z = 0.010008;
    this->mpf_taxels[6].vector.x = 0.008014;
    this->mpf_taxels[6].vector.y = 0.008283;
    this->mpf_taxels[6].vector.z = 0.009564;
    this->mpf_taxels[7].vector.x = 0.013002;
    this->mpf_taxels[7].vector.y = 0.008283;
    this->mpf_taxels[7].vector.z = 0.009122;
    this->mpf_taxels[8].vector.x = 0.019755;
    this->mpf_taxels[8].vector.y = 0.008283;
    this->mpf_taxels[8].vector.z = 0.008553;
    this->mpf_taxels[9].vector.x = 0.024652;
    this->mpf_taxels[9].vector.y = 0.008283;
    this->mpf_taxels[9].vector.z = 0.008093;
    this->mpf_taxels[10].vector.x = 0.029593;
    this->mpf_taxels[10].vector.y = 0.008283;
    this->mpf_taxels[10].vector.z = 0.007671;
    //Row2
    this->mpf_taxels[11].vector.x = -0.02393;
    this->mpf_taxels[11].vector.y = 0.008418;
    this->mpf_taxels[11].vector.z = 0.016317;
    this->mpf_taxels[12].vector.x = -0.018996;
    this->mpf_taxels[12].vector.y = 0.008418;
    this->mpf_taxels[12].vector.z = 0.015885;
    this->mpf_taxels[13].vector.x = -0.014108;
    this->mpf_taxels[13].vector.y = 0.008418;
    this->mpf_taxels[13].vector.z = 0.015457;
    this->mpf_taxels[14].vector.x = -0.007386;
    this->mpf_taxels[14].vector.y = 0.008418;
    this->mpf_taxels[14].vector.z = 0.014869;
    this->mpf_taxels[15].vector.x = -0.002074;
    this->mpf_taxels[15].vector.y = 0.008418;
    this->mpf_taxels[15].vector.z = 0.014454;
    this->mpf_taxels[16].vector.x = 0.003225;
    this->mpf_taxels[16].vector.y = 0.008418;
    this->mpf_taxels[16].vector.z = 0.013941;
    this->mpf_taxels[17].vector.x = 0.008358;
    this->mpf_taxels[17].vector.y = 0.008418;
    this->mpf_taxels[17].vector.z = 0.013497;
    this->mpf_taxels[18].vector.x = 0.013346;
    this->mpf_taxels[18].vector.y = 0.008418;
    this->mpf_taxels[18].vector.z = 0.013055;
    this->mpf_taxels[19].vector.x = 0.020099;
    this->mpf_taxels[19].vector.y = 0.008418;
    this->mpf_taxels[19].vector.z = 0.012486;
    this->mpf_taxels[20].vector.x = 0.025033;
    this->mpf_taxels[20].vector.y = 0.008418;
    this->mpf_taxels[20].vector.z = 0.012054;
    this->mpf_taxels[21].vector.x = 0.029937;
    this->mpf_taxels[21].vector.y = 0.008418;
    this->mpf_taxels[21].vector.z = 0.011604;
    //Row3
    this->mpf_taxels[22].vector.x = -0.023585;
    this->mpf_taxels[22].vector.y = 0.008329;
    this->mpf_taxels[22].vector.z = 0.02027;
    this->mpf_taxels[23].vector.x = -0.01865;
    this->mpf_taxels[23].vector.y = 0.008329;
    this->mpf_taxels[23].vector.z = 0.019838;
    this->mpf_taxels[24].vector.x = -0.013762;
    this->mpf_taxels[24].vector.y = 0.008329;
    this->mpf_taxels[24].vector.z = 0.019411;
    this->mpf_taxels[25].vector.x = -0.00704;
    this->mpf_taxels[25].vector.y = 0.008329;
    this->mpf_taxels[25].vector.z = 0.018822;
    this->mpf_taxels[26].vector.x = -0.001728;
    this->mpf_taxels[26].vector.y = 0.008329;
    this->mpf_taxels[26].vector.z = 0.018407;
    this->mpf_taxels[27].vector.x = 0.003571;
    this->mpf_taxels[27].vector.y = 0.008329;
    this->mpf_taxels[27].vector.z = 0.017894;
    this->mpf_taxels[28].vector.x = 0.008703;
    this->mpf_taxels[28].vector.y = 0.008329;
    this->mpf_taxels[28].vector.z = 0.01745;
    this->mpf_taxels[29].vector.x = 0.013692;
    this->mpf_taxels[29].vector.y = 0.008329;
    this->mpf_taxels[29].vector.z = 0.017009;
    this->mpf_taxels[30].vector.x = 0.020443;
    this->mpf_taxels[30].vector.y = 0.008329;
    this->mpf_taxels[30].vector.z = 0.016418;
    this->mpf_taxels[31].vector.x = 0.02534;
    this->mpf_taxels[31].vector.y = 0.008329;
    this->mpf_taxels[31].vector.z = 0.015957;
    this->mpf_taxels[32].vector.x = 0.030283;
    this->mpf_taxels[32].vector.y = 0.008329;
    this->mpf_taxels[32].vector.z = 0.015557;
    //Row4
    this->mpf_taxels[33].vector.x = -0.009048;
    this->mpf_taxels[33].vector.y = 0.007277;
    this->mpf_taxels[33].vector.z = 0.024677;
    this->mpf_taxels[34].vector.x = -0.00409;
    this->mpf_taxels[34].vector.y = 0.007277;
    this->mpf_taxels[34].vector.z = 0.024221;
    this->mpf_taxels[35].vector.x = 0.001613;
    this->mpf_taxels[35].vector.y = 0.007277;
    this->mpf_taxels[35].vector.z = 0.02383;
    this->mpf_taxels[36].vector.x = 0.0068;
    this->mpf_taxels[36].vector.y = 0.007277;
    this->mpf_taxels[36].vector.z = 0.023376;
    this->mpf_taxels[37].vector.x = 0.012334;
    this->mpf_taxels[37].vector.y = 0.007277;
    this->mpf_taxels[37].vector.z = 0.022812;
    this->mpf_taxels[38].vector.x = 0.017016;
    this->mpf_taxels[38].vector.y = 0.007277;
    this->mpf_taxels[38].vector.z = 0.022411;
    //Row5
    this->mpf_taxels[39].vector.x = -0.008665;
    this->mpf_taxels[39].vector.y = 0.005852;
    this->mpf_taxels[39].vector.z = 0.029059;
    this->mpf_taxels[40].vector.x = -0.003707;
    this->mpf_taxels[40].vector.y = 0.005852;
    this->mpf_taxels[40].vector.z = 0.028605;
    this->mpf_taxels[41].vector.x = 0.001996;
    this->mpf_taxels[41].vector.y = 0.005852;
    this->mpf_taxels[41].vector.z = 0.028207;
    this->mpf_taxels[42].vector.x = 0.007183;
    this->mpf_taxels[42].vector.y = 0.005852;
    this->mpf_taxels[42].vector.z = 0.027754;
    this->mpf_taxels[43].vector.x = 0.012715;
    this->mpf_taxels[43].vector.y = 0.005852;
    this->mpf_taxels[43].vector.z = 0.027174;
    this->mpf_taxels[44].vector.x = 0.017398;
    this->mpf_taxels[44].vector.y = 0.005852;
    this->mpf_taxels[44].vector.z = 0.026779;
    //Row6
    this->mpf_taxels[45].vector.x = -0.008457;
    this->mpf_taxels[45].vector.y = 0.001986;
    this->mpf_taxels[45].vector.z = 0.031433;
    this->mpf_taxels[46].vector.x = -0.003497;
    this->mpf_taxels[46].vector.y = 0.001986;
    this->mpf_taxels[46].vector.z = 0.030997;
    this->mpf_taxels[47].vector.x = 0.012924;
    this->mpf_taxels[47].vector.y = 0.001986;
    this->mpf_taxels[47].vector.z = 0.029562;
    this->mpf_taxels[48].vector.x = 0.017605;
    this->mpf_taxels[48].vector.y = 0.001986;
    this->mpf_taxels[48].vector.z = 0.029153;
    //Row7
    this->mpf_taxels[49].vector.x = -0.008405;
    this->mpf_taxels[49].vector.y = -0.002328;
    this->mpf_taxels[49].vector.z = 0.03203;
    this->mpf_taxels[50].vector.x = -0.003445;
    this->mpf_taxels[50].vector.y = -0.002328;
    this->mpf_taxels[50].vector.z = 0.031595;
    this->mpf_taxels[51].vector.x = 0.012976;
    this->mpf_taxels[51].vector.y = -0.002328;
    this->mpf_taxels[51].vector.z = 0.030159;
    this->mpf_taxels[52].vector.x = 0.017658;
    this->mpf_taxels[52].vector.y = -0.002328;
    this->mpf_taxels[52].vector.z = 0.029749;
    //Row8
    this->mpf_taxels[53].vector.x = -0.008396;
    this->mpf_taxels[53].vector.y = -0.006292;
    this->mpf_taxels[53].vector.z = 0.032132;
    this->mpf_taxels[54].vector.x = -0.003436;
    this->mpf_taxels[54].vector.y = -0.006292;
    this->mpf_taxels[54].vector.z = 0.031698;
    this->mpf_taxels[55].vector.x = 0.012987;
    this->mpf_taxels[55].vector.y = -0.006292;
    this->mpf_taxels[55].vector.z = 0.030278;
    this->mpf_taxels[56].vector.x = 0.017668;
    this->mpf_taxels[56].vector.y = -0.006292;
    this->mpf_taxels[56].vector.z = 0.029869;
    //Row9
    this->mpf_taxels[57].vector.x = -0.008394;
    this->mpf_taxels[57].vector.y = -0.010743;
    this->mpf_taxels[57].vector.z = 0.032153;
    this->mpf_taxels[58].vector.x = -0.003434;
    this->mpf_taxels[58].vector.y = -0.010743;
    this->mpf_taxels[58].vector.z = 0.031719;
    this->mpf_taxels[59].vector.x = 0.012987;
    this->mpf_taxels[59].vector.y = -0.010743;
    this->mpf_taxels[59].vector.z = 0.030282;
    this->mpf_taxels[60].vector.x = 0.017668;
    this->mpf_taxels[60].vector.y = -0.010743;
    this->mpf_taxels[60].vector.z = 0.029873;
    //Row10
    this->mpf_taxels[61].vector.x = -0.008394;
    this->mpf_taxels[61].vector.y = -0.015192;
    this->mpf_taxels[61].vector.z = 0.032153;
    this->mpf_taxels[62].vector.x = -0.003434;
    this->mpf_taxels[62].vector.y = -0.015192;
    this->mpf_taxels[62].vector.z = 0.031719;
    this->mpf_taxels[63].vector.x = 0.012987;
    this->mpf_taxels[63].vector.y = -0.015192;
    this->mpf_taxels[63].vector.z = 0.030282;
    this->mpf_taxels[64].vector.x = 0.017668;
    this->mpf_taxels[64].vector.y = -0.015192;
    this->mpf_taxels[64].vector.z = 0.029873;
    //Pinky Proximal Front

    //Row1
    this->ppf_taxels[0].vector.x = -0.024275;
    this->ppf_taxels[0].vector.y = 0.008283;
    this->ppf_taxels[0].vector.z = 0.012385;
    this->ppf_taxels[1].vector.x = -0.01934;
    this->ppf_taxels[1].vector.y = 0.008283;
    this->ppf_taxels[1].vector.z = 0.011953;
    this->ppf_taxels[2].vector.x = -0.014452;
    this->ppf_taxels[2].vector.y = 0.008283;
    this->ppf_taxels[2].vector.z = 0.011525;
    this->ppf_taxels[3].vector.x = -0.00773;
    this->ppf_taxels[3].vector.y = 0.008283;
    this->ppf_taxels[3].vector.z = 0.010937;
    this->ppf_taxels[4].vector.x = -0.002418;
    this->ppf_taxels[4].vector.y = 0.008283;
    this->ppf_taxels[4].vector.z = 0.010521;
    this->ppf_taxels[5].vector.x = 0.002881;
    this->ppf_taxels[5].vector.y = 0.008283;
    this->ppf_taxels[5].vector.z = 0.010008;
    this->ppf_taxels[6].vector.x = 0.008014;
    this->ppf_taxels[6].vector.y = 0.008283;
    this->ppf_taxels[6].vector.z = 0.009564;
    this->ppf_taxels[7].vector.x = 0.013002;
    this->ppf_taxels[7].vector.y = 0.008283;
    this->ppf_taxels[7].vector.z = 0.009122;
    this->ppf_taxels[8].vector.x = 0.019755;
    this->ppf_taxels[8].vector.y = 0.008283;
    this->ppf_taxels[8].vector.z = 0.008553;
    this->ppf_taxels[9].vector.x = 0.024652;
    this->ppf_taxels[9].vector.y = 0.008283;
    this->ppf_taxels[9].vector.z = 0.008093;
    this->ppf_taxels[10].vector.x = 0.029593;
    this->ppf_taxels[10].vector.y = 0.008283;
    this->ppf_taxels[10].vector.z = 0.007671;
    //Row2
    this->ppf_taxels[11].vector.x = -0.02393;
    this->ppf_taxels[11].vector.y = 0.008418;
    this->ppf_taxels[11].vector.z = 0.016317;
    this->ppf_taxels[12].vector.x = -0.018996;
    this->ppf_taxels[12].vector.y = 0.008418;
    this->ppf_taxels[12].vector.z = 0.015885;
    this->ppf_taxels[13].vector.x = -0.014108;
    this->ppf_taxels[13].vector.y = 0.008418;
    this->ppf_taxels[13].vector.z = 0.015457;
    this->ppf_taxels[14].vector.x = -0.007386;
    this->ppf_taxels[14].vector.y = 0.008418;
    this->ppf_taxels[14].vector.z = 0.014869;
    this->ppf_taxels[15].vector.x = -0.002074;
    this->ppf_taxels[15].vector.y = 0.008418;
    this->ppf_taxels[15].vector.z = 0.014454;
    this->ppf_taxels[16].vector.x = 0.003225;
    this->ppf_taxels[16].vector.y = 0.008418;
    this->ppf_taxels[16].vector.z = 0.013941;
    this->ppf_taxels[17].vector.x = 0.008358;
    this->ppf_taxels[17].vector.y = 0.008418;
    this->ppf_taxels[17].vector.z = 0.013497;
    this->ppf_taxels[18].vector.x = 0.013346;
    this->ppf_taxels[18].vector.y = 0.008418;
    this->ppf_taxels[18].vector.z = 0.013055;
    this->ppf_taxels[19].vector.x = 0.020099;
    this->ppf_taxels[19].vector.y = 0.008418;
    this->ppf_taxels[19].vector.z = 0.012486;
    this->ppf_taxels[20].vector.x = 0.025033;
    this->ppf_taxels[20].vector.y = 0.008418;
    this->ppf_taxels[20].vector.z = 0.012054;
    this->ppf_taxels[21].vector.x = 0.029937;
    this->ppf_taxels[21].vector.y = 0.008418;
    this->ppf_taxels[21].vector.z = 0.011604;
    //Row3
    this->ppf_taxels[22].vector.x = -0.023585;
    this->ppf_taxels[22].vector.y = 0.008329;
    this->ppf_taxels[22].vector.z = 0.02027;
    this->ppf_taxels[23].vector.x = -0.01865;
    this->ppf_taxels[23].vector.y = 0.008329;
    this->ppf_taxels[23].vector.z = 0.019838;
    this->ppf_taxels[24].vector.x = -0.013762;
    this->ppf_taxels[24].vector.y = 0.008329;
    this->ppf_taxels[24].vector.z = 0.019411;
    this->ppf_taxels[25].vector.x = -0.00704;
    this->ppf_taxels[25].vector.y = 0.008329;
    this->ppf_taxels[25].vector.z = 0.018822;
    this->ppf_taxels[26].vector.x = -0.001728;
    this->ppf_taxels[26].vector.y = 0.008329;
    this->ppf_taxels[26].vector.z = 0.018407;
    this->ppf_taxels[27].vector.x = 0.003571;
    this->ppf_taxels[27].vector.y = 0.008329;
    this->ppf_taxels[27].vector.z = 0.017894;
    this->ppf_taxels[28].vector.x = 0.008703;
    this->ppf_taxels[28].vector.y = 0.008329;
    this->ppf_taxels[28].vector.z = 0.01745;
    this->ppf_taxels[29].vector.x = 0.013692;
    this->ppf_taxels[29].vector.y = 0.008329;
    this->ppf_taxels[29].vector.z = 0.017009;
    this->ppf_taxels[30].vector.x = 0.020443;
    this->ppf_taxels[30].vector.y = 0.008329;
    this->ppf_taxels[30].vector.z = 0.016418;
    this->ppf_taxels[31].vector.x = 0.02534;
    this->ppf_taxels[31].vector.y = 0.008329;
    this->ppf_taxels[31].vector.z = 0.015957;
    this->ppf_taxels[32].vector.x = 0.030283;
    this->ppf_taxels[32].vector.y = 0.008329;
    this->ppf_taxels[32].vector.z = 0.015557;
    //Row4
    this->ppf_taxels[33].vector.x = -0.009048;
    this->ppf_taxels[33].vector.y = 0.007277;
    this->ppf_taxels[33].vector.z = 0.024677;
    this->ppf_taxels[34].vector.x = -0.00409;
    this->ppf_taxels[34].vector.y = 0.007277;
    this->ppf_taxels[34].vector.z = 0.024221;
    this->ppf_taxels[35].vector.x = 0.001613;
    this->ppf_taxels[35].vector.y = 0.007277;
    this->ppf_taxels[35].vector.z = 0.02383;
    this->ppf_taxels[36].vector.x = 0.0068;
    this->ppf_taxels[36].vector.y = 0.007277;
    this->ppf_taxels[36].vector.z = 0.023376;
    this->ppf_taxels[37].vector.x = 0.012334;
    this->ppf_taxels[37].vector.y = 0.007277;
    this->ppf_taxels[37].vector.z = 0.022812;
    this->ppf_taxels[38].vector.x = 0.017016;
    this->ppf_taxels[38].vector.y = 0.007277;
    this->ppf_taxels[38].vector.z = 0.022411;
    //Row5
    this->ppf_taxels[39].vector.x = -0.008665;
    this->ppf_taxels[39].vector.y = 0.005852;
    this->ppf_taxels[39].vector.z = 0.029059;
    this->ppf_taxels[40].vector.x = -0.003707;
    this->ppf_taxels[40].vector.y = 0.005852;
    this->ppf_taxels[40].vector.z = 0.028605;
    this->ppf_taxels[41].vector.x = 0.001996;
    this->ppf_taxels[41].vector.y = 0.005852;
    this->ppf_taxels[41].vector.z = 0.028207;
    this->ppf_taxels[42].vector.x = 0.007183;
    this->ppf_taxels[42].vector.y = 0.005852;
    this->ppf_taxels[42].vector.z = 0.027754;
    this->ppf_taxels[43].vector.x = 0.012715;
    this->ppf_taxels[43].vector.y = 0.005852;
    this->ppf_taxels[43].vector.z = 0.027174;
    this->ppf_taxels[44].vector.x = 0.017398;
    this->ppf_taxels[44].vector.y = 0.005852;
    this->ppf_taxels[44].vector.z = 0.026779;
    //Row6
    this->ppf_taxels[45].vector.x = -0.008457;
    this->ppf_taxels[45].vector.y = 0.001986;
    this->ppf_taxels[45].vector.z = 0.031433;
    this->ppf_taxels[46].vector.x = -0.003497;
    this->ppf_taxels[46].vector.y = 0.001986;
    this->ppf_taxels[46].vector.z = 0.030997;
    this->ppf_taxels[47].vector.x = 0.012924;
    this->ppf_taxels[47].vector.y = 0.001986;
    this->ppf_taxels[47].vector.z = 0.029562;
    this->ppf_taxels[48].vector.x = 0.017605;
    this->ppf_taxels[48].vector.y = 0.001986;
    this->ppf_taxels[48].vector.z = 0.029153;
    //Row7
    this->ppf_taxels[49].vector.x = -0.008405;
    this->ppf_taxels[49].vector.y = -0.002328;
    this->ppf_taxels[49].vector.z = 0.03203;
    this->ppf_taxels[50].vector.x = -0.003445;
    this->ppf_taxels[50].vector.y = -0.002328;
    this->ppf_taxels[50].vector.z = 0.031595;
    this->ppf_taxels[51].vector.x = 0.012976;
    this->ppf_taxels[51].vector.y = -0.002328;
    this->ppf_taxels[51].vector.z = 0.030159;
    this->ppf_taxels[52].vector.x = 0.017658;
    this->ppf_taxels[52].vector.y = -0.002328;
    this->ppf_taxels[52].vector.z = 0.029749;
    //Row8
    this->ppf_taxels[53].vector.x = -0.008396;
    this->ppf_taxels[53].vector.y = -0.006292;
    this->ppf_taxels[53].vector.z = 0.032132;
    this->ppf_taxels[54].vector.x = -0.003436;
    this->ppf_taxels[54].vector.y = -0.006292;
    this->ppf_taxels[54].vector.z = 0.031698;
    this->ppf_taxels[55].vector.x = 0.012987;
    this->ppf_taxels[55].vector.y = -0.006292;
    this->ppf_taxels[55].vector.z = 0.030278;
    this->ppf_taxels[56].vector.x = 0.017668;
    this->ppf_taxels[56].vector.y = -0.006292;
    this->ppf_taxels[56].vector.z = 0.029869;
    //Row9
    this->ppf_taxels[57].vector.x = -0.008394;
    this->ppf_taxels[57].vector.y = -0.010743;
    this->ppf_taxels[57].vector.z = 0.032153;
    this->ppf_taxels[58].vector.x = -0.003434;
    this->ppf_taxels[58].vector.y = -0.010743;
    this->ppf_taxels[58].vector.z = 0.031719;
    this->ppf_taxels[59].vector.x = 0.012987;
    this->ppf_taxels[59].vector.y = -0.010743;
    this->ppf_taxels[59].vector.z = 0.030282;
    this->ppf_taxels[60].vector.x = 0.017668;
    this->ppf_taxels[60].vector.y = -0.010743;
    this->ppf_taxels[60].vector.z = 0.029873;
    //Row10
    this->ppf_taxels[61].vector.x = -0.008394;
    this->ppf_taxels[61].vector.y = -0.015192;
    this->ppf_taxels[61].vector.z = 0.032153;
    this->ppf_taxels[62].vector.x = -0.003434;
    this->ppf_taxels[62].vector.y = -0.015192;
    this->ppf_taxels[62].vector.z = 0.031719;
    this->ppf_taxels[63].vector.x = 0.012987;
    this->ppf_taxels[63].vector.y = -0.015192;
    this->ppf_taxels[63].vector.z = 0.030282;
    this->ppf_taxels[64].vector.x = 0.017668;
    this->ppf_taxels[64].vector.y = -0.015192;
    this->ppf_taxels[64].vector.z = 0.029873;
    //Index Medial Front

    //Row1
    this->imf_taxels[0].vector.x = 0.018017;
    this->imf_taxels[0].vector.y = 0.01202;
    this->imf_taxels[0].vector.z = 0.011304;
    this->imf_taxels[1].vector.x = 0.009442;
    this->imf_taxels[1].vector.y = 0.01202;
    this->imf_taxels[1].vector.z = 0.01053;
    this->imf_taxels[2].vector.x = 0.003686;
    this->imf_taxels[2].vector.y = 0.01202;
    this->imf_taxels[2].vector.z = 0.010026;
    this->imf_taxels[3].vector.x = -0.001835;
    this->imf_taxels[3].vector.y = 0.01202;
    this->imf_taxels[3].vector.z = 0.009543;
    //Row2
    this->imf_taxels[4].vector.x = 0.017651;
    this->imf_taxels[4].vector.y = 0.012084;
    this->imf_taxels[4].vector.z = 0.01549;
    this->imf_taxels[5].vector.x = 0.009076;
    this->imf_taxels[5].vector.y = 0.012084;
    this->imf_taxels[5].vector.z = 0.014715;
    this->imf_taxels[6].vector.x = 0.00332;
    this->imf_taxels[6].vector.y = 0.012084;
    this->imf_taxels[6].vector.z = 0.014212;
    this->imf_taxels[7].vector.x = -0.002201;
    this->imf_taxels[7].vector.y = 0.012084;
    this->imf_taxels[7].vector.z = 0.013729;
    //Row3
    this->imf_taxels[8].vector.x = 0.017397;
    this->imf_taxels[8].vector.y = 0.012048;
    this->imf_taxels[8].vector.z = 0.019322;
    this->imf_taxels[9].vector.x = 0.008742;
    this->imf_taxels[9].vector.y = 0.012048;
    this->imf_taxels[9].vector.z = 0.01854;
    this->imf_taxels[10].vector.x = 0.002985;
    this->imf_taxels[10].vector.y = 0.012048;
    this->imf_taxels[10].vector.z = 0.018036;
    this->imf_taxels[11].vector.x = -0.002535;
    this->imf_taxels[11].vector.y = 0.012048;
    this->imf_taxels[11].vector.z = 0.017554;
    //Row4
    this->imf_taxels[12].vector.x = 0.008212;
    this->imf_taxels[12].vector.y = 0.011029;
    this->imf_taxels[12].vector.z = 0.024592;
    this->imf_taxels[13].vector.x = 0.002456;
    this->imf_taxels[13].vector.y = 0.011029;
    this->imf_taxels[13].vector.z = 0.024089;
    this->imf_taxels[14].vector.x = -0.003064;
    this->imf_taxels[14].vector.y = 0.011029;
    this->imf_taxels[14].vector.z = 0.023606;
    //Row5
    this->imf_taxels[15].vector.x = 0.007803;
    this->imf_taxels[15].vector.y = 0.009308;
    this->imf_taxels[15].vector.z = 0.029273;
    this->imf_taxels[16].vector.x = 0.002047;
    this->imf_taxels[16].vector.y = 0.009308;
    this->imf_taxels[16].vector.z = 0.028769;
    this->imf_taxels[17].vector.x = -0.003474;
    this->imf_taxels[17].vector.y = 0.009308;
    this->imf_taxels[17].vector.z = 0.028287;
    //Row6
    this->imf_taxels[18].vector.x = 0.007614;
    this->imf_taxels[18].vector.y = 0.004825;
    this->imf_taxels[18].vector.z = 0.031433;
    this->imf_taxels[19].vector.x = 0.001858;
    this->imf_taxels[19].vector.y = 0.004825;
    this->imf_taxels[19].vector.z = 0.03093;
    this->imf_taxels[20].vector.x = -0.003662;
    this->imf_taxels[20].vector.y = 0.004825;
    this->imf_taxels[20].vector.z = 0.030447;
    //Row7
    this->imf_taxels[21].vector.x = 0.007562;
    this->imf_taxels[21].vector.y = -0.000529;
    this->imf_taxels[21].vector.z = 0.032032;
    this->imf_taxels[22].vector.x = 0.001806;
    this->imf_taxels[22].vector.y = -0.000529;
    this->imf_taxels[22].vector.z = 0.031528;
    this->imf_taxels[23].vector.x = -0.003715;
    this->imf_taxels[23].vector.y = -0.000529;
    this->imf_taxels[23].vector.z = 0.031045;
    //Row8
    this->imf_taxels[24].vector.x = 0.007542;
    this->imf_taxels[24].vector.y = -0.007143;
    this->imf_taxels[24].vector.z = 0.032078;
    this->imf_taxels[25].vector.x = -0.015341;
    this->imf_taxels[25].vector.y = -0.007143;
    this->imf_taxels[25].vector.z = 0.030076;
    this->imf_taxels[26].vector.x = -0.022805;
    this->imf_taxels[26].vector.y = -0.007143;
    this->imf_taxels[26].vector.z = 0.029423;
    //Middle Medial Front

    //Row1
    this->mmf_taxels[0].vector.x = 0.018017;
    this->mmf_taxels[0].vector.y = 0.01202;
    this->mmf_taxels[0].vector.z = 0.011304;
    this->mmf_taxels[1].vector.x = 0.009442;
    this->mmf_taxels[1].vector.y = 0.01202;
    this->mmf_taxels[1].vector.z = 0.01053;
    this->mmf_taxels[2].vector.x = 0.003686;
    this->mmf_taxels[2].vector.y = 0.01202;
    this->mmf_taxels[2].vector.z = 0.010026;
    this->mmf_taxels[3].vector.x = -0.001835;
    this->mmf_taxels[3].vector.y = 0.01202;
    this->mmf_taxels[3].vector.z = 0.009543;
    //Row2
    this->mmf_taxels[4].vector.x = 0.017651;
    this->mmf_taxels[4].vector.y = 0.012084;
    this->mmf_taxels[4].vector.z = 0.01549;
    this->mmf_taxels[5].vector.x = 0.009076;
    this->mmf_taxels[5].vector.y = 0.012084;
    this->mmf_taxels[5].vector.z = 0.014715;
    this->mmf_taxels[6].vector.x = 0.00332;
    this->mmf_taxels[6].vector.y = 0.012084;
    this->mmf_taxels[6].vector.z = 0.014212;
    this->mmf_taxels[7].vector.x = -0.002201;
    this->mmf_taxels[7].vector.y = 0.012084;
    this->mmf_taxels[7].vector.z = 0.013729;
    //Row3
    this->mmf_taxels[8].vector.x = 0.017397;
    this->mmf_taxels[8].vector.y = 0.012048;
    this->mmf_taxels[8].vector.z = 0.019322;
    this->mmf_taxels[9].vector.x = 0.008742;
    this->mmf_taxels[9].vector.y = 0.012048;
    this->mmf_taxels[9].vector.z = 0.01854;
    this->mmf_taxels[10].vector.x = 0.002985;
    this->mmf_taxels[10].vector.y = 0.012048;
    this->mmf_taxels[10].vector.z = 0.018036;
    this->mmf_taxels[11].vector.x = -0.002535;
    this->mmf_taxels[11].vector.y = 0.012048;
    this->mmf_taxels[11].vector.z = 0.017554;
    //Row4
    this->mmf_taxels[12].vector.x = 0.008212;
    this->mmf_taxels[12].vector.y = 0.011029;
    this->mmf_taxels[12].vector.z = 0.024592;
    this->mmf_taxels[13].vector.x = 0.002456;
    this->mmf_taxels[13].vector.y = 0.011029;
    this->mmf_taxels[13].vector.z = 0.024089;
    this->mmf_taxels[14].vector.x = -0.003064;
    this->mmf_taxels[14].vector.y = 0.011029;
    this->mmf_taxels[14].vector.z = 0.023606;
    //Row5
    this->mmf_taxels[15].vector.x = 0.007803;
    this->mmf_taxels[15].vector.y = 0.009308;
    this->mmf_taxels[15].vector.z = 0.029273;
    this->mmf_taxels[16].vector.x = 0.002047;
    this->mmf_taxels[16].vector.y = 0.009308;
    this->mmf_taxels[16].vector.z = 0.028769;
    this->mmf_taxels[17].vector.x = -0.003474;
    this->mmf_taxels[17].vector.y = 0.009308;
    this->mmf_taxels[17].vector.z = 0.028287;
    //Row6
    this->mmf_taxels[18].vector.x = 0.007614;
    this->mmf_taxels[18].vector.y = 0.004825;
    this->mmf_taxels[18].vector.z = 0.031433;
    this->mmf_taxels[19].vector.x = 0.001858;
    this->mmf_taxels[19].vector.y = 0.004825;
    this->mmf_taxels[19].vector.z = 0.03093;
    this->mmf_taxels[20].vector.x = -0.003662;
    this->mmf_taxels[20].vector.y = 0.004825;
    this->mmf_taxels[20].vector.z = 0.030447;
    //Row7
    this->mmf_taxels[21].vector.x = 0.007562;
    this->mmf_taxels[21].vector.y = -0.000529;
    this->mmf_taxels[21].vector.z = 0.032032;
    this->mmf_taxels[22].vector.x = 0.001806;
    this->mmf_taxels[22].vector.y = -0.000529;
    this->mmf_taxels[22].vector.z = 0.031528;
    this->mmf_taxels[23].vector.x = -0.003715;
    this->mmf_taxels[23].vector.y = -0.000529;
    this->mmf_taxels[23].vector.z = 0.031045;
    //Row8
    this->mmf_taxels[24].vector.x = 0.007542;
    this->mmf_taxels[24].vector.y = -0.007143;
    this->mmf_taxels[24].vector.z = 0.032078;
    this->mmf_taxels[25].vector.x = -0.015341;
    this->mmf_taxels[25].vector.y = -0.007143;
    this->mmf_taxels[25].vector.z = 0.030076;
    this->mmf_taxels[26].vector.x = -0.022805;
    this->mmf_taxels[26].vector.y = -0.007143;
    this->mmf_taxels[26].vector.z = 0.029423;
    //Pinky Medial Front

    //Row1
    this->pmf_taxels[0].vector.x = 0.018017;
    this->pmf_taxels[0].vector.y = 0.01202;
    this->pmf_taxels[0].vector.z = 0.011304;
    this->pmf_taxels[1].vector.x = 0.009442;
    this->pmf_taxels[1].vector.y = 0.01202;
    this->pmf_taxels[1].vector.z = 0.01053;
    this->pmf_taxels[2].vector.x = 0.003686;
    this->pmf_taxels[2].vector.y = 0.01202;
    this->pmf_taxels[2].vector.z = 0.010026;
    this->pmf_taxels[3].vector.x = -0.001835;
    this->pmf_taxels[3].vector.y = 0.01202;
    this->pmf_taxels[3].vector.z = 0.009543;
    //Row2
    this->pmf_taxels[4].vector.x = 0.017651;
    this->pmf_taxels[4].vector.y = 0.012084;
    this->pmf_taxels[4].vector.z = 0.01549;
    this->pmf_taxels[5].vector.x = 0.009076;
    this->pmf_taxels[5].vector.y = 0.012084;
    this->pmf_taxels[5].vector.z = 0.014715;
    this->pmf_taxels[6].vector.x = 0.00332;
    this->pmf_taxels[6].vector.y = 0.012084;
    this->pmf_taxels[6].vector.z = 0.014212;
    this->pmf_taxels[7].vector.x = -0.002201;
    this->pmf_taxels[7].vector.y = 0.012084;
    this->pmf_taxels[7].vector.z = 0.013729;
    //Row3
    this->pmf_taxels[8].vector.x = 0.017397;
    this->pmf_taxels[8].vector.y = 0.012048;
    this->pmf_taxels[8].vector.z = 0.019322;
    this->pmf_taxels[9].vector.x = 0.008742;
    this->pmf_taxels[9].vector.y = 0.012048;
    this->pmf_taxels[9].vector.z = 0.01854;
    this->pmf_taxels[10].vector.x = 0.002985;
    this->pmf_taxels[10].vector.y = 0.012048;
    this->pmf_taxels[10].vector.z = 0.018036;
    this->pmf_taxels[11].vector.x = -0.002535;
    this->pmf_taxels[11].vector.y = 0.012048;
    this->pmf_taxels[11].vector.z = 0.017554;
    //Row4
    this->pmf_taxels[12].vector.x = 0.008212;
    this->pmf_taxels[12].vector.y = 0.011029;
    this->pmf_taxels[12].vector.z = 0.024592;
    this->pmf_taxels[13].vector.x = 0.002456;
    this->pmf_taxels[13].vector.y = 0.011029;
    this->pmf_taxels[13].vector.z = 0.024089;
    this->pmf_taxels[14].vector.x = -0.003064;
    this->pmf_taxels[14].vector.y = 0.011029;
    this->pmf_taxels[14].vector.z = 0.023606;
    //Row5
    this->pmf_taxels[15].vector.x = 0.007803;
    this->pmf_taxels[15].vector.y = 0.009308;
    this->pmf_taxels[15].vector.z = 0.029273;
    this->pmf_taxels[16].vector.x = 0.002047;
    this->pmf_taxels[16].vector.y = 0.009308;
    this->pmf_taxels[16].vector.z = 0.028769;
    this->pmf_taxels[17].vector.x = -0.003474;
    this->pmf_taxels[17].vector.y = 0.009308;
    this->pmf_taxels[17].vector.z = 0.028287;
    //Row6
    this->pmf_taxels[18].vector.x = 0.007614;
    this->pmf_taxels[18].vector.y = 0.004825;
    this->pmf_taxels[18].vector.z = 0.031433;
    this->pmf_taxels[19].vector.x = 0.001858;
    this->pmf_taxels[19].vector.y = 0.004825;
    this->pmf_taxels[19].vector.z = 0.03093;
    this->pmf_taxels[20].vector.x = -0.003662;
    this->pmf_taxels[20].vector.y = 0.004825;
    this->pmf_taxels[20].vector.z = 0.030447;
    //Row7
    this->pmf_taxels[21].vector.x = 0.007562;
    this->pmf_taxels[21].vector.y = -0.000529;
    this->pmf_taxels[21].vector.z = 0.032032;
    this->pmf_taxels[22].vector.x = 0.001806;
    this->pmf_taxels[22].vector.y = -0.000529;
    this->pmf_taxels[22].vector.z = 0.031528;
    this->pmf_taxels[23].vector.x = -0.003715;
    this->pmf_taxels[23].vector.y = -0.000529;
    this->pmf_taxels[23].vector.z = 0.031045;
    //Row8
    this->pmf_taxels[24].vector.x = 0.007542;
    this->pmf_taxels[24].vector.y = -0.007143;
    this->pmf_taxels[24].vector.z = 0.032078;
    this->pmf_taxels[25].vector.x = -0.015341;
    this->pmf_taxels[25].vector.y = -0.007143;
    this->pmf_taxels[25].vector.z = 0.030076;
    this->pmf_taxels[26].vector.x = -0.022805;
    this->pmf_taxels[26].vector.y = -0.007143;
    this->pmf_taxels[26].vector.z = 0.029423;
    //Thumb Fingertip

    //Straight wrapp up
    //Row1
    this->tft_taxels[0].vector.x = 0.015337;
    this->tft_taxels[0].vector.y = -0.006754;
    this->tft_taxels[0].vector.z = -0.00513;
    this->tft_taxels[1].vector.x = 0.015239;
    this->tft_taxels[1].vector.y = 0.001668;
    this->tft_taxels[1].vector.z = -0.005106;
    this->tft_taxels[2].vector.x = 0.015289;
    this->tft_taxels[2].vector.y = 0.010175;
    this->tft_taxels[2].vector.z = -0.004527;
    this->tft_taxels[3].vector.x = 0.016136;
    this->tft_taxels[3].vector.y = 0.0139;
    this->tft_taxels[3].vector.z = 0.00504;
    this->tft_taxels[4].vector.x = 0.016712;
    this->tft_taxels[4].vector.y = 0.0139;
    this->tft_taxels[4].vector.z = 0.011619;
    this->tft_taxels[5].vector.x = 0.017287;
    this->tft_taxels[5].vector.y = 0.0139;
    this->tft_taxels[5].vector.z = 0.018198;
    this->tft_taxels[6].vector.x = 0.018274;
    this->tft_taxels[6].vector.y = 0.011735;
    this->tft_taxels[6].vector.z = 0.028435;
    this->tft_taxels[7].vector.x = 0.018342;
    this->tft_taxels[7].vector.y = 0.00314;
    this->tft_taxels[7].vector.z = 0.029222;
    this->tft_taxels[8].vector.x = 0.018251;
    this->tft_taxels[8].vector.y = -0.005069;
    this->tft_taxels[8].vector.z = 0.02932;
    //Row2
    this->tft_taxels[9].vector.x = 0.021649;
    this->tft_taxels[9].vector.y = -0.006754;
    this->tft_taxels[9].vector.z = -0.005683;
    this->tft_taxels[10].vector.x = 0.021551;
    this->tft_taxels[10].vector.y = 0.001668;
    this->tft_taxels[10].vector.z = -0.005659;
    this->tft_taxels[11].vector.x = 0.021601;
    this->tft_taxels[11].vector.y = 0.010175;
    this->tft_taxels[11].vector.z = -0.005079;
    this->tft_taxels[12].vector.x = 0.022448;
    this->tft_taxels[12].vector.y = 0.0139;
    this->tft_taxels[12].vector.z = 0.004488;
    this->tft_taxels[13].vector.x = 0.023024;
    this->tft_taxels[13].vector.y = 0.0139;
    this->tft_taxels[13].vector.z = 0.011067;
    this->tft_taxels[14].vector.x = 0.023599;
    this->tft_taxels[14].vector.y = 0.0139;
    this->tft_taxels[14].vector.z = 0.017646;
    this->tft_taxels[15].vector.x = 0.024586;
    this->tft_taxels[15].vector.y = 0.011735;
    this->tft_taxels[15].vector.z = 0.027883;
    this->tft_taxels[16].vector.x = 0.024654;
    this->tft_taxels[16].vector.y = 0.00314;
    this->tft_taxels[16].vector.z = 0.028669;
    this->tft_taxels[17].vector.x = 0.024563;
    this->tft_taxels[17].vector.y = -0.005069;
    this->tft_taxels[17].vector.z = 0.028768;
    //Row3
    this->tft_taxels[18].vector.x = 0.027724;
    this->tft_taxels[18].vector.y = -0.001668;
    this->tft_taxels[18].vector.z = -0.006199;
    this->tft_taxels[19].vector.x = 0.027775;
    this->tft_taxels[19].vector.y = 0.010175;
    this->tft_taxels[19].vector.z = -0.005619;
    this->tft_taxels[20].vector.x = 0.028622;
    this->tft_taxels[20].vector.y = 0.0139;
    this->tft_taxels[20].vector.z = 0.003948;
    this->tft_taxels[21].vector.x = 0.029197;
    this->tft_taxels[21].vector.y = 0.0139;
    this->tft_taxels[21].vector.z = 0.010527;
    this->tft_taxels[22].vector.x = 0.029773;
    this->tft_taxels[22].vector.y = 0.0139;
    this->tft_taxels[22].vector.z = 0.017106;
    this->tft_taxels[23].vector.x = 0.030759;
    this->tft_taxels[23].vector.y = 0.011735;
    this->tft_taxels[23].vector.z = 0.027343;
    this->tft_taxels[24].vector.x = 0.030828;
    this->tft_taxels[24].vector.y = 0.00314;
    this->tft_taxels[24].vector.z = 0.028129;
    //Row4
    this->tft_taxels[25].vector.x = 0.033486;
    this->tft_taxels[25].vector.y = 0.010175;
    this->tft_taxels[25].vector.z = -0.006199;
    this->tft_taxels[26].vector.x = 0.034333;
    this->tft_taxels[26].vector.y = 0.0139;
    this->tft_taxels[26].vector.z = 0.003448;
    this->tft_taxels[27].vector.x = 0.034909;
    this->tft_taxels[27].vector.y = 0.0139;
    this->tft_taxels[27].vector.z = 0.010027;
    this->tft_taxels[28].vector.x = 0.035484;
    this->tft_taxels[28].vector.y = 0.0139;
    this->tft_taxels[28].vector.z = 0.016606;
    this->tft_taxels[29].vector.x = 0.03647;
    this->tft_taxels[29].vector.y = 0.011735;
    this->tft_taxels[29].vector.z = 0.026843;
    // Curved Face
    //Row1
    this->tft_taxels[30].vector.x = 0.043079;
    this->tft_taxels[30].vector.y = 0.013442;
    this->tft_taxels[30].vector.z = -0.000058;
    this->tft_taxels[31].vector.x = 0.043633;
    this->tft_taxels[31].vector.y = 0.013442;
    this->tft_taxels[31].vector.z = 0.006267;
    this->tft_taxels[32].vector.x = 0.044186;
    this->tft_taxels[32].vector.y = 0.013442;
    this->tft_taxels[32].vector.z = 0.012593;
    this->tft_taxels[33].vector.x = 0.04474;
    this->tft_taxels[33].vector.y = 0.013442;
    this->tft_taxels[33].vector.z = 0.018919;
    //Row2
    this->tft_taxels[34].vector.x = 0.04288;
    this->tft_taxels[34].vector.y = 0.007274;
    this->tft_taxels[34].vector.z = -0.000041;
    this->tft_taxels[35].vector.x = 0.043434;
    this->tft_taxels[35].vector.y = 0.007274;
    this->tft_taxels[35].vector.z = 0.006285;
    this->tft_taxels[36].vector.x = 0.043987;
    this->tft_taxels[36].vector.y = 0.007274;
    this->tft_taxels[36].vector.z = 0.012611;
    this->tft_taxels[37].vector.x = 0.044541;
    this->tft_taxels[37].vector.y = 0.007274;
    this->tft_taxels[37].vector.z = 0.018936;
    //Row3
    this->tft_taxels[38].vector.x = 0.040359;
    this->tft_taxels[38].vector.y = 0.002132;
    this->tft_taxels[38].vector.z = -0.0000482;
    this->tft_taxels[39].vector.x = 0.040912;
    this->tft_taxels[39].vector.y = 0.002132;
    this->tft_taxels[39].vector.z = 0.006808;
    this->tft_taxels[40].vector.x = 0.041465;
    this->tft_taxels[40].vector.y = 0.002132;
    this->tft_taxels[40].vector.z = 0.013133;
    this->tft_taxels[41].vector.x = 0.042019;
    this->tft_taxels[41].vector.y = 0.002132;
    this->tft_taxels[41].vector.z = 0.019459;
    //Row4
    this->tft_taxels[42].vector.x = 0.037108;
    this->tft_taxels[42].vector.y = -0.002795;
    this->tft_taxels[42].vector.z = -0.000766;
    this->tft_taxels[43].vector.x = 0.037662;
    this->tft_taxels[43].vector.y = -0.002795;
    this->tft_taxels[43].vector.z = 0.007092;
    this->tft_taxels[44].vector.x = 0.038215;
    this->tft_taxels[44].vector.y = -0.002795;
    this->tft_taxels[44].vector.z = 0.013418;
    this->tft_taxels[45].vector.x = 0.038769;
    this->tft_taxels[45].vector.y = -0.002795;
    this->tft_taxels[45].vector.z = 0.019744;
    //Row5
    this->tft_taxels[46].vector.x = 0.033168;
    this->tft_taxels[46].vector.y = -0.007189;
    this->tft_taxels[46].vector.z = 0.001111;
    this->tft_taxels[47].vector.x = 0.033721;
    this->tft_taxels[47].vector.y = -0.007189;
    this->tft_taxels[47].vector.z = 0.007437;
    this->tft_taxels[48].vector.x = 0.034275;
    this->tft_taxels[48].vector.y = -0.007189;
    this->tft_taxels[48].vector.z = 0.013762;
    this->tft_taxels[49].vector.x = 0.034828;
    this->tft_taxels[49].vector.y = -0.007189;
    this->tft_taxels[49].vector.z = 0.020088;
    //Row6
    this->tft_taxels[50].vector.x = 0.028594;
    this->tft_taxels[50].vector.y = -0.010913;
    this->tft_taxels[50].vector.z = 0.001511;
    this->tft_taxels[51].vector.x = 0.029148;
    this->tft_taxels[51].vector.y = -0.010913;
    this->tft_taxels[51].vector.z = 0.007837;
    this->tft_taxels[52].vector.x = 0.029701;
    this->tft_taxels[52].vector.y = -0.010913;
    this->tft_taxels[52].vector.z = 0.014163;
    this->tft_taxels[53].vector.x = 0.030255;
    this->tft_taxels[53].vector.y = -0.010913;
    this->tft_taxels[53].vector.z = 0.020488;
    //Row7
    this->tft_taxels[54].vector.x = 0.023521;
    this->tft_taxels[54].vector.y = -0.01392;
    this->tft_taxels[54].vector.z = 0.001955;
    this->tft_taxels[55].vector.x = 0.024075;
    this->tft_taxels[55].vector.y = -0.01392;
    this->tft_taxels[55].vector.z = 0.008281;
    this->tft_taxels[56].vector.x = 0.024628;
    this->tft_taxels[56].vector.y = -0.01392;
    this->tft_taxels[56].vector.z = 0.014606;
    this->tft_taxels[57].vector.x = 0.025181;
    this->tft_taxels[57].vector.y = -0.01392;
    this->tft_taxels[57].vector.z = 0.020932;
    //Row8
    this->tft_taxels[58].vector.x = 0.018044;
    this->tft_taxels[58].vector.y = -0.016096;
    this->tft_taxels[58].vector.z = 0.002434;
    this->tft_taxels[59].vector.x = 0.018598;
    this->tft_taxels[59].vector.y = -0.016096;
    this->tft_taxels[59].vector.z = 0.00876;
    this->tft_taxels[60].vector.x = 0.019151;
    this->tft_taxels[60].vector.y = -0.016096;
    this->tft_taxels[60].vector.z = 0.015086;
    this->tft_taxels[61].vector.x = 0.019705;
    this->tft_taxels[61].vector.y = -0.016096;
    this->tft_taxels[61].vector.z = 0.021411;
    //Row9
    this->tft_taxels[62].vector.x = 0.012303;
    this->tft_taxels[62].vector.y = -0.017413;
    this->tft_taxels[62].vector.z = 0.002936;
    this->tft_taxels[63].vector.x = 0.012857;
    this->tft_taxels[63].vector.y = -0.017413;
    this->tft_taxels[63].vector.z = 0.009262;
    this->tft_taxels[64].vector.x = 0.01341;
    this->tft_taxels[64].vector.y = -0.017413;
    this->tft_taxels[64].vector.z = 0.015588;
    this->tft_taxels[65].vector.x = 0.013963;
    this->tft_taxels[65].vector.y = -0.017413;
    this->tft_taxels[65].vector.z = 0.021914;

    //Index Fingertip

    //Straight wrapp up
    //Row1
    this->ift_taxels[0].vector.x = 0.015337;
    this->ift_taxels[0].vector.y = -0.006754;
    this->ift_taxels[0].vector.z = -0.00513;
    this->ift_taxels[1].vector.x = 0.015239;
    this->ift_taxels[1].vector.y = 0.001668;
    this->ift_taxels[1].vector.z = -0.005106;
    this->ift_taxels[2].vector.x = 0.015289;
    this->ift_taxels[2].vector.y = 0.010175;
    this->ift_taxels[2].vector.z = -0.004527;
    this->ift_taxels[3].vector.x = 0.016136;
    this->ift_taxels[3].vector.y = 0.0139;
    this->ift_taxels[3].vector.z = 0.00504;
    this->ift_taxels[4].vector.x = 0.016712;
    this->ift_taxels[4].vector.y = 0.0139;
    this->ift_taxels[4].vector.z = 0.011619;
    this->ift_taxels[5].vector.x = 0.017287;
    this->ift_taxels[5].vector.y = 0.0139;
    this->ift_taxels[5].vector.z = 0.018198;
    this->ift_taxels[6].vector.x = 0.018274;
    this->ift_taxels[6].vector.y = 0.011735;
    this->ift_taxels[6].vector.z = 0.028435;
    this->ift_taxels[7].vector.x = 0.018342;
    this->ift_taxels[7].vector.y = 0.00314;
    this->ift_taxels[7].vector.z = 0.029222;
    this->ift_taxels[8].vector.x = 0.018251;
    this->ift_taxels[8].vector.y = -0.005069;
    this->ift_taxels[8].vector.z = 0.02932;
    //Row2
    this->ift_taxels[9].vector.x = 0.021649;
    this->ift_taxels[9].vector.y = -0.006754;
    this->ift_taxels[9].vector.z = -0.005683;
    this->ift_taxels[10].vector.x = 0.021551;
    this->ift_taxels[10].vector.y = 0.001668;
    this->ift_taxels[10].vector.z = -0.005659;
    this->ift_taxels[11].vector.x = 0.021601;
    this->ift_taxels[11].vector.y = 0.010175;
    this->ift_taxels[11].vector.z = -0.005079;
    this->ift_taxels[12].vector.x = 0.022448;
    this->ift_taxels[12].vector.y = 0.0139;
    this->ift_taxels[12].vector.z = 0.004488;
    this->ift_taxels[13].vector.x = 0.023024;
    this->ift_taxels[13].vector.y = 0.0139;
    this->ift_taxels[13].vector.z = 0.011067;
    this->ift_taxels[14].vector.x = 0.023599;
    this->ift_taxels[14].vector.y = 0.0139;
    this->ift_taxels[14].vector.z = 0.017646;
    this->ift_taxels[15].vector.x = 0.024586;
    this->ift_taxels[15].vector.y = 0.011735;
    this->ift_taxels[15].vector.z = 0.027883;
    this->ift_taxels[16].vector.x = 0.024654;
    this->ift_taxels[16].vector.y = 0.00314;
    this->ift_taxels[16].vector.z = 0.028669;
    this->ift_taxels[17].vector.x = 0.024563;
    this->ift_taxels[17].vector.y = -0.005069;
    this->ift_taxels[17].vector.z = 0.028768;
    //Row3
    this->ift_taxels[18].vector.x = 0.027724;
    this->ift_taxels[18].vector.y = -0.001668;
    this->ift_taxels[18].vector.z = -0.006199;
    this->ift_taxels[19].vector.x = 0.027775;
    this->ift_taxels[19].vector.y = 0.010175;
    this->ift_taxels[19].vector.z = -0.005619;
    this->ift_taxels[20].vector.x = 0.028622;
    this->ift_taxels[20].vector.y = 0.0139;
    this->ift_taxels[20].vector.z = 0.003948;
    this->ift_taxels[21].vector.x = 0.029197;
    this->ift_taxels[21].vector.y = 0.0139;
    this->ift_taxels[21].vector.z = 0.010527;
    this->ift_taxels[22].vector.x = 0.029773;
    this->ift_taxels[22].vector.y = 0.0139;
    this->ift_taxels[22].vector.z = 0.017106;
    this->ift_taxels[23].vector.x = 0.030759;
    this->ift_taxels[23].vector.y = 0.011735;
    this->ift_taxels[23].vector.z = 0.027343;
    this->ift_taxels[24].vector.x = 0.030828;
    this->ift_taxels[24].vector.y = 0.00314;
    this->ift_taxels[24].vector.z = 0.028129;
    //Row4
    this->ift_taxels[25].vector.x = 0.033486;
    this->ift_taxels[25].vector.y = 0.010175;
    this->ift_taxels[25].vector.z = -0.006199;
    this->ift_taxels[26].vector.x = 0.034333;
    this->ift_taxels[26].vector.y = 0.0139;
    this->ift_taxels[26].vector.z = 0.003448;
    this->ift_taxels[27].vector.x = 0.034909;
    this->ift_taxels[27].vector.y = 0.0139;
    this->ift_taxels[27].vector.z = 0.010027;
    this->ift_taxels[28].vector.x = 0.035484;
    this->ift_taxels[28].vector.y = 0.0139;
    this->ift_taxels[28].vector.z = 0.016606;
    this->ift_taxels[29].vector.x = 0.03647;
    this->ift_taxels[29].vector.y = 0.011735;
    this->ift_taxels[29].vector.z = 0.026843;
    // Curved Face
    //Row1
    this->ift_taxels[30].vector.x = 0.043079;
    this->ift_taxels[30].vector.y = 0.013442;
    this->ift_taxels[30].vector.z = -0.000058;
    this->ift_taxels[31].vector.x = 0.043633;
    this->ift_taxels[31].vector.y = 0.013442;
    this->ift_taxels[31].vector.z = 0.006267;
    this->ift_taxels[32].vector.x = 0.044186;
    this->ift_taxels[32].vector.y = 0.013442;
    this->ift_taxels[32].vector.z = 0.012593;
    this->ift_taxels[33].vector.x = 0.04474;
    this->ift_taxels[33].vector.y = 0.013442;
    this->ift_taxels[33].vector.z = 0.018919;
    //Row2
    this->ift_taxels[34].vector.x = 0.04288;
    this->ift_taxels[34].vector.y = 0.007274;
    this->ift_taxels[34].vector.z = -0.000041;
    this->ift_taxels[35].vector.x = 0.043434;
    this->ift_taxels[35].vector.y = 0.007274;
    this->ift_taxels[35].vector.z = 0.006285;
    this->ift_taxels[36].vector.x = 0.043987;
    this->ift_taxels[36].vector.y = 0.007274;
    this->ift_taxels[36].vector.z = 0.012611;
    this->ift_taxels[37].vector.x = 0.044541;
    this->ift_taxels[37].vector.y = 0.007274;
    this->ift_taxels[37].vector.z = 0.018936;
    //Row3
    this->ift_taxels[38].vector.x = 0.040359;
    this->ift_taxels[38].vector.y = 0.002132;
    this->ift_taxels[38].vector.z = -0.0000482;
    this->ift_taxels[39].vector.x = 0.040912;
    this->ift_taxels[39].vector.y = 0.002132;
    this->ift_taxels[39].vector.z = 0.006808;
    this->ift_taxels[40].vector.x = 0.041465;
    this->ift_taxels[40].vector.y = 0.002132;
    this->ift_taxels[40].vector.z = 0.013133;
    this->ift_taxels[41].vector.x = 0.042019;
    this->ift_taxels[41].vector.y = 0.002132;
    this->ift_taxels[41].vector.z = 0.019459;
    //Row4
    this->ift_taxels[42].vector.x = 0.037108;
    this->ift_taxels[42].vector.y = -0.002795;
    this->ift_taxels[42].vector.z = -0.000766;
    this->ift_taxels[43].vector.x = 0.037662;
    this->ift_taxels[43].vector.y = -0.002795;
    this->ift_taxels[43].vector.z = 0.007092;
    this->ift_taxels[44].vector.x = 0.038215;
    this->ift_taxels[44].vector.y = -0.002795;
    this->ift_taxels[44].vector.z = 0.013418;
    this->ift_taxels[45].vector.x = 0.038769;
    this->ift_taxels[45].vector.y = -0.002795;
    this->ift_taxels[45].vector.z = 0.019744;
    //Row5
    this->ift_taxels[46].vector.x = 0.033168;
    this->ift_taxels[46].vector.y = -0.007189;
    this->ift_taxels[46].vector.z = 0.001111;
    this->ift_taxels[47].vector.x = 0.033721;
    this->ift_taxels[47].vector.y = -0.007189;
    this->ift_taxels[47].vector.z = 0.007437;
    this->ift_taxels[48].vector.x = 0.034275;
    this->ift_taxels[48].vector.y = -0.007189;
    this->ift_taxels[48].vector.z = 0.013762;
    this->ift_taxels[49].vector.x = 0.034828;
    this->ift_taxels[49].vector.y = -0.007189;
    this->ift_taxels[49].vector.z = 0.020088;
    //Row6
    this->ift_taxels[50].vector.x = 0.028594;
    this->ift_taxels[50].vector.y = -0.010913;
    this->ift_taxels[50].vector.z = 0.001511;
    this->ift_taxels[51].vector.x = 0.029148;
    this->ift_taxels[51].vector.y = -0.010913;
    this->ift_taxels[51].vector.z = 0.007837;
    this->ift_taxels[52].vector.x = 0.029701;
    this->ift_taxels[52].vector.y = -0.010913;
    this->ift_taxels[52].vector.z = 0.014163;
    this->ift_taxels[53].vector.x = 0.030255;
    this->ift_taxels[53].vector.y = -0.010913;
    this->ift_taxels[53].vector.z = 0.020488;
    //Row7
    this->ift_taxels[54].vector.x = 0.023521;
    this->ift_taxels[54].vector.y = -0.01392;
    this->ift_taxels[54].vector.z = 0.001955;
    this->ift_taxels[55].vector.x = 0.024075;
    this->ift_taxels[55].vector.y = -0.01392;
    this->ift_taxels[55].vector.z = 0.008281;
    this->ift_taxels[56].vector.x = 0.024628;
    this->ift_taxels[56].vector.y = -0.01392;
    this->ift_taxels[56].vector.z = 0.014606;
    this->ift_taxels[57].vector.x = 0.025181;
    this->ift_taxels[57].vector.y = -0.01392;
    this->ift_taxels[57].vector.z = 0.020932;
    //Row8
    this->ift_taxels[58].vector.x = 0.018044;
    this->ift_taxels[58].vector.y = -0.016096;
    this->ift_taxels[58].vector.z = 0.002434;
    this->ift_taxels[59].vector.x = 0.018598;
    this->ift_taxels[59].vector.y = -0.016096;
    this->ift_taxels[59].vector.z = 0.00876;
    this->ift_taxels[60].vector.x = 0.019151;
    this->ift_taxels[60].vector.y = -0.016096;
    this->ift_taxels[60].vector.z = 0.015086;
    this->ift_taxels[61].vector.x = 0.019705;
    this->ift_taxels[61].vector.y = -0.016096;
    this->ift_taxels[61].vector.z = 0.021411;
    //Row9
    this->ift_taxels[62].vector.x = 0.012303;
    this->ift_taxels[62].vector.y = -0.017413;
    this->ift_taxels[62].vector.z = 0.002936;
    this->ift_taxels[63].vector.x = 0.012857;
    this->ift_taxels[63].vector.y = -0.017413;
    this->ift_taxels[63].vector.z = 0.009262;
    this->ift_taxels[64].vector.x = 0.01341;
    this->ift_taxels[64].vector.y = -0.017413;
    this->ift_taxels[64].vector.z = 0.015588;
    this->ift_taxels[65].vector.x = 0.013963;
    this->ift_taxels[65].vector.y = -0.017413;
    this->ift_taxels[65].vector.z = 0.021914;

    //Middle Fingertip

    //Straight wrapp up
    //Row1
    this->mft_taxels[0].vector.x = 0.015337;
    this->mft_taxels[0].vector.y = -0.006754;
    this->mft_taxels[0].vector.z = -0.00513;
    this->mft_taxels[1].vector.x = 0.015239;
    this->mft_taxels[1].vector.y = 0.001668;
    this->mft_taxels[1].vector.z = -0.005106;
    this->mft_taxels[2].vector.x = 0.015289;
    this->mft_taxels[2].vector.y = 0.010175;
    this->mft_taxels[2].vector.z = -0.004527;
    this->mft_taxels[3].vector.x = 0.016136;
    this->mft_taxels[3].vector.y = 0.0139;
    this->mft_taxels[3].vector.z = 0.00504;
    this->mft_taxels[4].vector.x = 0.016712;
    this->mft_taxels[4].vector.y = 0.0139;
    this->mft_taxels[4].vector.z = 0.011619;
    this->mft_taxels[5].vector.x = 0.017287;
    this->mft_taxels[5].vector.y = 0.0139;
    this->mft_taxels[5].vector.z = 0.018198;
    this->mft_taxels[6].vector.x = 0.018274;
    this->mft_taxels[6].vector.y = 0.011735;
    this->mft_taxels[6].vector.z = 0.028435;
    this->mft_taxels[7].vector.x = 0.018342;
    this->mft_taxels[7].vector.y = 0.00314;
    this->mft_taxels[7].vector.z = 0.029222;
    this->mft_taxels[8].vector.x = 0.018251;
    this->mft_taxels[8].vector.y = -0.005069;
    this->mft_taxels[8].vector.z = 0.02932;
    //Row2
    this->mft_taxels[9].vector.x = 0.021649;
    this->mft_taxels[9].vector.y = -0.006754;
    this->mft_taxels[9].vector.z = -0.005683;
    this->mft_taxels[10].vector.x = 0.021551;
    this->mft_taxels[10].vector.y = 0.001668;
    this->mft_taxels[10].vector.z = -0.005659;
    this->mft_taxels[11].vector.x = 0.021601;
    this->mft_taxels[11].vector.y = 0.010175;
    this->mft_taxels[11].vector.z = -0.005079;
    this->mft_taxels[12].vector.x = 0.022448;
    this->mft_taxels[12].vector.y = 0.0139;
    this->mft_taxels[12].vector.z = 0.004488;
    this->mft_taxels[13].vector.x = 0.023024;
    this->mft_taxels[13].vector.y = 0.0139;
    this->mft_taxels[13].vector.z = 0.011067;
    this->mft_taxels[14].vector.x = 0.023599;
    this->mft_taxels[14].vector.y = 0.0139;
    this->mft_taxels[14].vector.z = 0.017646;
    this->mft_taxels[15].vector.x = 0.024586;
    this->mft_taxels[15].vector.y = 0.011735;
    this->mft_taxels[15].vector.z = 0.027883;
    this->mft_taxels[16].vector.x = 0.024654;
    this->mft_taxels[16].vector.y = 0.00314;
    this->mft_taxels[16].vector.z = 0.028669;
    this->mft_taxels[17].vector.x = 0.024563;
    this->mft_taxels[17].vector.y = -0.005069;
    this->mft_taxels[17].vector.z = 0.028768;
    //Row3
    this->mft_taxels[18].vector.x = 0.027724;
    this->mft_taxels[18].vector.y = -0.001668;
    this->mft_taxels[18].vector.z = -0.006199;
    this->mft_taxels[19].vector.x = 0.027775;
    this->mft_taxels[19].vector.y = 0.010175;
    this->mft_taxels[19].vector.z = -0.005619;
    this->mft_taxels[20].vector.x = 0.028622;
    this->mft_taxels[20].vector.y = 0.0139;
    this->mft_taxels[20].vector.z = 0.003948;
    this->mft_taxels[21].vector.x = 0.029197;
    this->mft_taxels[21].vector.y = 0.0139;
    this->mft_taxels[21].vector.z = 0.010527;
    this->mft_taxels[22].vector.x = 0.029773;
    this->mft_taxels[22].vector.y = 0.0139;
    this->mft_taxels[22].vector.z = 0.017106;
    this->mft_taxels[23].vector.x = 0.030759;
    this->mft_taxels[23].vector.y = 0.011735;
    this->mft_taxels[23].vector.z = 0.027343;
    this->mft_taxels[24].vector.x = 0.030828;
    this->mft_taxels[24].vector.y = 0.00314;
    this->mft_taxels[24].vector.z = 0.028129;
    //Row4
    this->mft_taxels[25].vector.x = 0.033486;
    this->mft_taxels[25].vector.y = 0.010175;
    this->mft_taxels[25].vector.z = -0.006199;
    this->mft_taxels[26].vector.x = 0.034333;
    this->mft_taxels[26].vector.y = 0.0139;
    this->mft_taxels[26].vector.z = 0.003448;
    this->mft_taxels[27].vector.x = 0.034909;
    this->mft_taxels[27].vector.y = 0.0139;
    this->mft_taxels[27].vector.z = 0.010027;
    this->mft_taxels[28].vector.x = 0.035484;
    this->mft_taxels[28].vector.y = 0.0139;
    this->mft_taxels[28].vector.z = 0.016606;
    this->mft_taxels[29].vector.x = 0.03647;
    this->mft_taxels[29].vector.y = 0.011735;
    this->mft_taxels[29].vector.z = 0.026843;
    // Curved Face
    //Row1
    this->mft_taxels[30].vector.x = 0.043079;
    this->mft_taxels[30].vector.y = 0.013442;
    this->mft_taxels[30].vector.z = -0.000058;
    this->mft_taxels[31].vector.x = 0.043633;
    this->mft_taxels[31].vector.y = 0.013442;
    this->mft_taxels[31].vector.z = 0.006267;
    this->mft_taxels[32].vector.x = 0.044186;
    this->mft_taxels[32].vector.y = 0.013442;
    this->mft_taxels[32].vector.z = 0.012593;
    this->mft_taxels[33].vector.x = 0.04474;
    this->mft_taxels[33].vector.y = 0.013442;
    this->mft_taxels[33].vector.z = 0.018919;
    //Row2
    this->mft_taxels[34].vector.x = 0.04288;
    this->mft_taxels[34].vector.y = 0.007274;
    this->mft_taxels[34].vector.z = -0.000041;
    this->mft_taxels[35].vector.x = 0.043434;
    this->mft_taxels[35].vector.y = 0.007274;
    this->mft_taxels[35].vector.z = 0.006285;
    this->mft_taxels[36].vector.x = 0.043987;
    this->mft_taxels[36].vector.y = 0.007274;
    this->mft_taxels[36].vector.z = 0.012611;
    this->mft_taxels[37].vector.x = 0.044541;
    this->mft_taxels[37].vector.y = 0.007274;
    this->mft_taxels[37].vector.z = 0.018936;
    //Row3
    this->mft_taxels[38].vector.x = 0.040359;
    this->mft_taxels[38].vector.y = 0.002132;
    this->mft_taxels[38].vector.z = -0.0000482;
    this->mft_taxels[39].vector.x = 0.040912;
    this->mft_taxels[39].vector.y = 0.002132;
    this->mft_taxels[39].vector.z = 0.006808;
    this->mft_taxels[40].vector.x = 0.041465;
    this->mft_taxels[40].vector.y = 0.002132;
    this->mft_taxels[40].vector.z = 0.013133;
    this->mft_taxels[41].vector.x = 0.042019;
    this->mft_taxels[41].vector.y = 0.002132;
    this->mft_taxels[41].vector.z = 0.019459;
    //Row4
    this->mft_taxels[42].vector.x = 0.037108;
    this->mft_taxels[42].vector.y = -0.002795;
    this->mft_taxels[42].vector.z = -0.000766;
    this->mft_taxels[43].vector.x = 0.037662;
    this->mft_taxels[43].vector.y = -0.002795;
    this->mft_taxels[43].vector.z = 0.007092;
    this->mft_taxels[44].vector.x = 0.038215;
    this->mft_taxels[44].vector.y = -0.002795;
    this->mft_taxels[44].vector.z = 0.013418;
    this->mft_taxels[45].vector.x = 0.038769;
    this->mft_taxels[45].vector.y = -0.002795;
    this->mft_taxels[45].vector.z = 0.019744;
    //Row5
    this->mft_taxels[46].vector.x = 0.033168;
    this->mft_taxels[46].vector.y = -0.007189;
    this->mft_taxels[46].vector.z = 0.001111;
    this->mft_taxels[47].vector.x = 0.033721;
    this->mft_taxels[47].vector.y = -0.007189;
    this->mft_taxels[47].vector.z = 0.007437;
    this->mft_taxels[48].vector.x = 0.034275;
    this->mft_taxels[48].vector.y = -0.007189;
    this->mft_taxels[48].vector.z = 0.013762;
    this->mft_taxels[49].vector.x = 0.034828;
    this->mft_taxels[49].vector.y = -0.007189;
    this->mft_taxels[49].vector.z = 0.020088;
    //Row6
    this->mft_taxels[50].vector.x = 0.028594;
    this->mft_taxels[50].vector.y = -0.010913;
    this->mft_taxels[50].vector.z = 0.001511;
    this->mft_taxels[51].vector.x = 0.029148;
    this->mft_taxels[51].vector.y = -0.010913;
    this->mft_taxels[51].vector.z = 0.007837;
    this->mft_taxels[52].vector.x = 0.029701;
    this->mft_taxels[52].vector.y = -0.010913;
    this->mft_taxels[52].vector.z = 0.014163;
    this->mft_taxels[53].vector.x = 0.030255;
    this->mft_taxels[53].vector.y = -0.010913;
    this->mft_taxels[53].vector.z = 0.020488;
    //Row7
    this->mft_taxels[54].vector.x = 0.023521;
    this->mft_taxels[54].vector.y = -0.01392;
    this->mft_taxels[54].vector.z = 0.001955;
    this->mft_taxels[55].vector.x = 0.024075;
    this->mft_taxels[55].vector.y = -0.01392;
    this->mft_taxels[55].vector.z = 0.008281;
    this->mft_taxels[56].vector.x = 0.024628;
    this->mft_taxels[56].vector.y = -0.01392;
    this->mft_taxels[56].vector.z = 0.014606;
    this->mft_taxels[57].vector.x = 0.025181;
    this->mft_taxels[57].vector.y = -0.01392;
    this->mft_taxels[57].vector.z = 0.020932;
    //Row8
    this->mft_taxels[58].vector.x = 0.018044;
    this->mft_taxels[58].vector.y = -0.016096;
    this->mft_taxels[58].vector.z = 0.002434;
    this->mft_taxels[59].vector.x = 0.018598;
    this->mft_taxels[59].vector.y = -0.016096;
    this->mft_taxels[59].vector.z = 0.00876;
    this->mft_taxels[60].vector.x = 0.019151;
    this->mft_taxels[60].vector.y = -0.016096;
    this->mft_taxels[60].vector.z = 0.015086;
    this->mft_taxels[61].vector.x = 0.019705;
    this->mft_taxels[61].vector.y = -0.016096;
    this->mft_taxels[61].vector.z = 0.021411;
    //Row9
    this->mft_taxels[62].vector.x = 0.012303;
    this->mft_taxels[62].vector.y = -0.017413;
    this->mft_taxels[62].vector.z = 0.002936;
    this->mft_taxels[63].vector.x = 0.012857;
    this->mft_taxels[63].vector.y = -0.017413;
    this->mft_taxels[63].vector.z = 0.009262;
    this->mft_taxels[64].vector.x = 0.01341;
    this->mft_taxels[64].vector.y = -0.017413;
    this->mft_taxels[64].vector.z = 0.015588;
    this->mft_taxels[65].vector.x = 0.013963;
    this->mft_taxels[65].vector.y = -0.017413;
    this->mft_taxels[65].vector.z = 0.021914;
    //Pinky Fingertip

    //Straight wrapp up
    //Row1
    this->pft_taxels[0].vector.x = 0.015337;
    this->pft_taxels[0].vector.y = -0.006754;
    this->pft_taxels[0].vector.z = -0.00513;
    this->pft_taxels[1].vector.x = 0.015239;
    this->pft_taxels[1].vector.y = 0.001668;
    this->pft_taxels[1].vector.z = -0.005106;
    this->pft_taxels[2].vector.x = 0.015289;
    this->pft_taxels[2].vector.y = 0.010175;
    this->pft_taxels[2].vector.z = -0.004527;
    this->pft_taxels[3].vector.x = 0.016136;
    this->pft_taxels[3].vector.y = 0.0139;
    this->pft_taxels[3].vector.z = 0.00504;
    this->pft_taxels[4].vector.x = 0.016712;
    this->pft_taxels[4].vector.y = 0.0139;
    this->pft_taxels[4].vector.z = 0.011619;
    this->pft_taxels[5].vector.x = 0.017287;
    this->pft_taxels[5].vector.y = 0.0139;
    this->pft_taxels[5].vector.z = 0.018198;
    this->pft_taxels[6].vector.x = 0.018274;
    this->pft_taxels[6].vector.y = 0.011735;
    this->pft_taxels[6].vector.z = 0.028435;
    this->pft_taxels[7].vector.x = 0.018342;
    this->pft_taxels[7].vector.y = 0.00314;
    this->pft_taxels[7].vector.z = 0.029222;
    this->pft_taxels[8].vector.x = 0.018251;
    this->pft_taxels[8].vector.y = -0.005069;
    this->pft_taxels[8].vector.z = 0.02932;
    //Row2
    this->pft_taxels[9].vector.x = 0.021649;
    this->pft_taxels[9].vector.y = -0.006754;
    this->pft_taxels[9].vector.z = -0.005683;
    this->pft_taxels[10].vector.x = 0.021551;
    this->pft_taxels[10].vector.y = 0.001668;
    this->pft_taxels[10].vector.z = -0.005659;
    this->pft_taxels[11].vector.x = 0.021601;
    this->pft_taxels[11].vector.y = 0.010175;
    this->pft_taxels[11].vector.z = -0.005079;
    this->pft_taxels[12].vector.x = 0.022448;
    this->pft_taxels[12].vector.y = 0.0139;
    this->pft_taxels[12].vector.z = 0.004488;
    this->pft_taxels[13].vector.x = 0.023024;
    this->pft_taxels[13].vector.y = 0.0139;
    this->pft_taxels[13].vector.z = 0.011067;
    this->pft_taxels[14].vector.x = 0.023599;
    this->pft_taxels[14].vector.y = 0.0139;
    this->pft_taxels[14].vector.z = 0.017646;
    this->pft_taxels[15].vector.x = 0.024586;
    this->pft_taxels[15].vector.y = 0.011735;
    this->pft_taxels[15].vector.z = 0.027883;
    this->pft_taxels[16].vector.x = 0.024654;
    this->pft_taxels[16].vector.y = 0.00314;
    this->pft_taxels[16].vector.z = 0.028669;
    this->pft_taxels[17].vector.x = 0.024563;
    this->pft_taxels[17].vector.y = -0.005069;
    this->pft_taxels[17].vector.z = 0.028768;
    //Row3
    this->pft_taxels[18].vector.x = 0.027724;
    this->pft_taxels[18].vector.y = -0.001668;
    this->pft_taxels[18].vector.z = -0.006199;
    this->pft_taxels[19].vector.x = 0.027775;
    this->pft_taxels[19].vector.y = 0.010175;
    this->pft_taxels[19].vector.z = -0.005619;
    this->pft_taxels[20].vector.x = 0.028622;
    this->pft_taxels[20].vector.y = 0.0139;
    this->pft_taxels[20].vector.z = 0.003948;
    this->pft_taxels[21].vector.x = 0.029197;
    this->pft_taxels[21].vector.y = 0.0139;
    this->pft_taxels[21].vector.z = 0.010527;
    this->pft_taxels[22].vector.x = 0.029773;
    this->pft_taxels[22].vector.y = 0.0139;
    this->pft_taxels[22].vector.z = 0.017106;
    this->pft_taxels[23].vector.x = 0.030759;
    this->pft_taxels[23].vector.y = 0.011735;
    this->pft_taxels[23].vector.z = 0.027343;
    this->pft_taxels[24].vector.x = 0.030828;
    this->pft_taxels[24].vector.y = 0.00314;
    this->pft_taxels[24].vector.z = 0.028129;
    //Row4
    this->pft_taxels[25].vector.x = 0.033486;
    this->pft_taxels[25].vector.y = 0.010175;
    this->pft_taxels[25].vector.z = -0.006199;
    this->pft_taxels[26].vector.x = 0.034333;
    this->pft_taxels[26].vector.y = 0.0139;
    this->pft_taxels[26].vector.z = 0.003448;
    this->pft_taxels[27].vector.x = 0.034909;
    this->pft_taxels[27].vector.y = 0.0139;
    this->pft_taxels[27].vector.z = 0.010027;
    this->pft_taxels[28].vector.x = 0.035484;
    this->pft_taxels[28].vector.y = 0.0139;
    this->pft_taxels[28].vector.z = 0.016606;
    this->pft_taxels[29].vector.x = 0.03647;
    this->pft_taxels[29].vector.y = 0.011735;
    this->pft_taxels[29].vector.z = 0.026843;
    // Curved Face
    //Row1
    this->pft_taxels[30].vector.x = 0.043079;
    this->pft_taxels[30].vector.y = 0.013442;
    this->pft_taxels[30].vector.z = -0.000058;
    this->pft_taxels[31].vector.x = 0.043633;
    this->pft_taxels[31].vector.y = 0.013442;
    this->pft_taxels[31].vector.z = 0.006267;
    this->pft_taxels[32].vector.x = 0.044186;
    this->pft_taxels[32].vector.y = 0.013442;
    this->pft_taxels[32].vector.z = 0.012593;
    this->pft_taxels[33].vector.x = 0.04474;
    this->pft_taxels[33].vector.y = 0.013442;
    this->pft_taxels[33].vector.z = 0.018919;
    //Row2
    this->pft_taxels[34].vector.x = 0.04288;
    this->pft_taxels[34].vector.y = 0.007274;
    this->pft_taxels[34].vector.z = -0.000041;
    this->pft_taxels[35].vector.x = 0.043434;
    this->pft_taxels[35].vector.y = 0.007274;
    this->pft_taxels[35].vector.z = 0.006285;
    this->pft_taxels[36].vector.x = 0.043987;
    this->pft_taxels[36].vector.y = 0.007274;
    this->pft_taxels[36].vector.z = 0.012611;
    this->pft_taxels[37].vector.x = 0.044541;
    this->pft_taxels[37].vector.y = 0.007274;
    this->pft_taxels[37].vector.z = 0.018936;
    //Row3
    this->pft_taxels[38].vector.x = 0.040359;
    this->pft_taxels[38].vector.y = 0.002132;
    this->pft_taxels[38].vector.z = -0.0000482;
    this->pft_taxels[39].vector.x = 0.040912;
    this->pft_taxels[39].vector.y = 0.002132;
    this->pft_taxels[39].vector.z = 0.006808;
    this->pft_taxels[40].vector.x = 0.041465;
    this->pft_taxels[40].vector.y = 0.002132;
    this->pft_taxels[40].vector.z = 0.013133;
    this->pft_taxels[41].vector.x = 0.042019;
    this->pft_taxels[41].vector.y = 0.002132;
    this->pft_taxels[41].vector.z = 0.019459;
    //Row4
    this->pft_taxels[42].vector.x = 0.037108;
    this->pft_taxels[42].vector.y = -0.002795;
    this->pft_taxels[42].vector.z = -0.000766;
    this->pft_taxels[43].vector.x = 0.037662;
    this->pft_taxels[43].vector.y = -0.002795;
    this->pft_taxels[43].vector.z = 0.007092;
    this->pft_taxels[44].vector.x = 0.038215;
    this->pft_taxels[44].vector.y = -0.002795;
    this->pft_taxels[44].vector.z = 0.013418;
    this->pft_taxels[45].vector.x = 0.038769;
    this->pft_taxels[45].vector.y = -0.002795;
    this->pft_taxels[45].vector.z = 0.019744;
    //Row5
    this->pft_taxels[46].vector.x = 0.033168;
    this->pft_taxels[46].vector.y = -0.007189;
    this->pft_taxels[46].vector.z = 0.001111;
    this->pft_taxels[47].vector.x = 0.033721;
    this->pft_taxels[47].vector.y = -0.007189;
    this->pft_taxels[47].vector.z = 0.007437;
    this->pft_taxels[48].vector.x = 0.034275;
    this->pft_taxels[48].vector.y = -0.007189;
    this->pft_taxels[48].vector.z = 0.013762;
    this->pft_taxels[49].vector.x = 0.034828;
    this->pft_taxels[49].vector.y = -0.007189;
    this->pft_taxels[49].vector.z = 0.020088;
    //Row6
    this->pft_taxels[50].vector.x = 0.028594;
    this->pft_taxels[50].vector.y = -0.010913;
    this->pft_taxels[50].vector.z = 0.001511;
    this->pft_taxels[51].vector.x = 0.029148;
    this->pft_taxels[51].vector.y = -0.010913;
    this->pft_taxels[51].vector.z = 0.007837;
    this->pft_taxels[52].vector.x = 0.029701;
    this->pft_taxels[52].vector.y = -0.010913;
    this->pft_taxels[52].vector.z = 0.014163;
    this->pft_taxels[53].vector.x = 0.030255;
    this->pft_taxels[53].vector.y = -0.010913;
    this->pft_taxels[53].vector.z = 0.020488;
    //Row7
    this->pft_taxels[54].vector.x = 0.023521;
    this->pft_taxels[54].vector.y = -0.01392;
    this->pft_taxels[54].vector.z = 0.001955;
    this->pft_taxels[55].vector.x = 0.024075;
    this->pft_taxels[55].vector.y = -0.01392;
    this->pft_taxels[55].vector.z = 0.008281;
    this->pft_taxels[56].vector.x = 0.024628;
    this->pft_taxels[56].vector.y = -0.01392;
    this->pft_taxels[56].vector.z = 0.014606;
    this->pft_taxels[57].vector.x = 0.025181;
    this->pft_taxels[57].vector.y = -0.01392;
    this->pft_taxels[57].vector.z = 0.020932;
    //Row8
    this->pft_taxels[58].vector.x = 0.018044;
    this->pft_taxels[58].vector.y = -0.016096;
    this->pft_taxels[58].vector.z = 0.002434;
    this->pft_taxels[59].vector.x = 0.018598;
    this->pft_taxels[59].vector.y = -0.016096;
    this->pft_taxels[59].vector.z = 0.00876;
    this->pft_taxels[60].vector.x = 0.019151;
    this->pft_taxels[60].vector.y = -0.016096;
    this->pft_taxels[60].vector.z = 0.015086;
    this->pft_taxels[61].vector.x = 0.019705;
    this->pft_taxels[61].vector.y = -0.016096;
    this->pft_taxels[61].vector.z = 0.021411;
    //Row9
    this->pft_taxels[62].vector.x = 0.012303;
    this->pft_taxels[62].vector.y = -0.017413;
    this->pft_taxels[62].vector.z = 0.002936;
    this->pft_taxels[63].vector.x = 0.012857;
    this->pft_taxels[63].vector.y = -0.017413;
    this->pft_taxels[63].vector.z = 0.009262;
    this->pft_taxels[64].vector.x = 0.01341;
    this->pft_taxels[64].vector.y = -0.017413;
    this->pft_taxels[64].vector.z = 0.015588;
    this->pft_taxels[65].vector.x = 0.013963;
    this->pft_taxels[65].vector.y = -0.017413;
    this->pft_taxels[65].vector.z = 0.021914;

    // Filling the taxels_map
    this->taxels_map["boh"] = this->boh_taxels;
    this->taxels_map["palm"] = this->palm_taxels;
    this->taxels_map["ipb"] = this->ipb_taxels;
    this->taxels_map["mpb"] = this->mpb_taxels;
    this->taxels_map["ppb"] = this->ppb_taxels;
    this->taxels_map["ipf"] = this->ipf_taxels;
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
    this->transformed_taxels_map["ipf"] = this->trans_ipf_taxels;
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
    int counter = 0;
    if (this->start == true)
    {

        // Getting the transformation from the brackets to the world frame at the initial home position of the allegro hand
        for (auto const& [key, val] : this->taxels_map)
        {
            if(tfBuffer.canTransform("world",key+"_bracket", ros::Time(0)))
            {
                counter++;
                this->transformStamped = tfBuffer.lookupTransform("world",key+"_bracket", ros::Time(0));
                this->home_transforms[key] = this->transformStamped;

                /*
                std::cout<< key << " CONTACTS: " << std::endl;
                std::cout<< "Transformation:"<< "\n"
                         <<"q0: "<< this->transformStamped.transform.rotation.x<<"\n"
                         <<"q1: "<< this->transformStamped.transform.rotation.y<<"\n"
                         <<"q2: "<< this->transformStamped.transform.rotation.z<<"\n"
                         <<"q3: "<< this->transformStamped.transform.rotation.w<<"\n"
                         <<"dx: "<< this->transformStamped.transform.translation.x<<"\n"
                         <<"dy: "<< this->transformStamped.transform.translation.y<<"\n"
                         <<"dz: "<< this->transformStamped.transform.translation.z<< std::endl;
                std::vector<std::vector<float>> my_homogeneous_matrix = this->homo_matrix(this->transformStamped);
                std::cout<< "Homogeneous Transformation:"<< "\n"
                    <<"["<< my_homogeneous_matrix[0][0]<<"   "<<my_homogeneous_matrix[0][1]<<"   "<<my_homogeneous_matrix[0][2]<<"   "<<my_homogeneous_matrix[0][3]<<"\n"
                         << my_homogeneous_matrix[1][0]<<"   "<<my_homogeneous_matrix[1][1]<<"   "<<my_homogeneous_matrix[1][2]<<"   "<<my_homogeneous_matrix[1][3]<<"\n"
                         << my_homogeneous_matrix[2][0]<<"   "<<my_homogeneous_matrix[2][1]<<"   "<<my_homogeneous_matrix[2][2]<<"   "<<my_homogeneous_matrix[2][3]<<"\n"
                         << my_homogeneous_matrix[3][0]<<"   "<<my_homogeneous_matrix[3][1]<<"   "<<my_homogeneous_matrix[3][2]<<"   "<<my_homogeneous_matrix[3][3]<<"]"<<std::endl;
                */
            }
            if(counter == 20) //After placing all the sensors on the hand, this number should be replaced
            {                 // by this->sensor_count
                this->start = false;
                this->end = ros::Time::now().toSec();
                std::cout << "Time needed to extract the home pose transformations (Sec): " << end-begin << std::endl;
            }
        }

    }
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
//        for (auto const& [key, val] : this->taxels_map)
        {
           this->target_sensor = key;
           // Getting the transformation from the brackets to the world frame using tf2
           if(tfBuffer.canTransform("world",key+"_bracket", ros::Time(0)))
           {
               this->transformStamped = tfBuffer.lookupTransform("world",key+"_bracket", ros::Time(0));
           /*
               std::cout<< "Transformation:"<< "\n"
                        <<"q0: "<< this->transformStamped.transform.rotation.x<<"\n"
                        <<"q1: "<< this->transformStamped.transform.rotation.y<<"\n"
                        <<"q2: "<< this->transformStamped.transform.rotation.z<<"\n"
                        <<"q3: "<< this->transformStamped.transform.rotation.w<<"\n"
                        <<"dx: "<< this->transformStamped.transform.translation.x<<"\n"
                        <<"dy: "<< this->transformStamped.transform.translation.y<<"\n"
                        <<"dz: "<< this->transformStamped.transform.translation.z<< std::endl;
               std::vector<std::vector<float>> my_homogeneous_matrix = this->homo_matrix(this->transformStamped);
               std::cout<< "Homogeneous Transformation:"<< "\n"
                   <<"["<< my_homogeneous_matrix[0][0]<<"   "<<my_homogeneous_matrix[0][1]<<"   "<<my_homogeneous_matrix[0][2]<<"   "<<my_homogeneous_matrix[0][3]<<"\n"
                        << my_homogeneous_matrix[1][0]<<"   "<<my_homogeneous_matrix[1][1]<<"   "<<my_homogeneous_matrix[1][2]<<"   "<<my_homogeneous_matrix[1][3]<<"\n"
                        << my_homogeneous_matrix[2][0]<<"   "<<my_homogeneous_matrix[2][1]<<"   "<<my_homogeneous_matrix[2][2]<<"   "<<my_homogeneous_matrix[2][3]<<"\n"
                        << my_homogeneous_matrix[3][0]<<"   "<<my_homogeneous_matrix[3][1]<<"   "<<my_homogeneous_matrix[3][2]<<"   "<<my_homogeneous_matrix[3][3]<<"]"<<std::endl;
           */
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

               //Adjusting the positions of the raw contact points
               for (unsigned int i = 0; i < this->raw_contacts_map[key].size() ; ++i)
               {
/*
                   std::cout<<"Original Contact Locations: " <<raw_contacts_map[key][i][0] <<"  "
                                                            <<raw_contacts_map[key][i][1] <<"  "
                                                            <<raw_contacts_map[key][i][2] <<"  "<<std::endl;
*/
                   this->v3s.vector.x = raw_contacts_map[key][i][0];
                   this->v3s.vector.y = raw_contacts_map[key][i][1];
                   this->v3s.vector.z = raw_contacts_map[key][i][2];
                   this->raw_contacts_map[key][i] = rawhomotrans(this->home_transforms[key],this->transformStamped,this->v3s);
/*
                   std::cout<<"Updated Contact Locations: " <<raw_contacts_map[key][i][0] <<"  "
                                                            <<raw_contacts_map[key][i][1] <<"  "
                                                            <<raw_contacts_map[key][i][2] <<"  "<<std::endl;
*/
               }
           }
           //********************************Testing Area************************************************
           std::cout<<"Total number of contact points: " << val.size() << std::endl;
           for (unsigned int i = 0; i < this->raw_contacts_map[key].size() ; ++i)
           {
               //Comparing the location of each contact point of a sensor with the locations of the same sensor's taxels
               for (unsigned int j = 0; j < this->transformed_taxels_map[key].size() ; ++j)
               {
                   /*
                   std::cout << "Distance: "
                             << this->dist(val[i],this->transformed_taxels_map[key][j])
                             << std::endl;
                   */
                   if(this->dist(raw_contacts_map[key][i],this->transformed_taxels_map[key][j]) < dist_threshold &&
                           std::find(this->filtered_contact_positions_v.begin(),this->filtered_contact_positions_v.end(),
                           this->transformed_taxels_map[key][j]) == this->filtered_contact_positions_v.end())
                   {
                       //Replace the contact point by the correspondent taxel(s)
                       std::cout << "Distance: "
                                 << this->dist(raw_contacts_map[key][i],this->transformed_taxels_map[key][j])
                                 << std::endl;
                       std::cout<<"Pushed contact vector: " << transformed_taxels_map[key][j][0] << "  "
                                << transformed_taxels_map[key][j][1]<<"  "<< transformed_taxels_map[key][j][2]
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
            for (unsigned int i = 0; i < raw_contacts_map[key].size() ; ++i)
            {
            fprintf(this->fp, "%f,%f,%f \n", raw_contacts_map[key][i][0],raw_contacts_map[key][i][1],raw_contacts_map[key][i][2]);
            }

        }
        //Flushing the FILTERED contact locations into a .csv file
        for (auto const& [key, val] : this->filtered_contacts_map)
 //       for (auto const& [key, val] : this->transformed_taxels_map)
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
      this->begin = ros::Time::now().toSec();
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
      this->sensor_count = _parent->GetSensorCount();
      std::cout << "The number of attached sensors is: " + std::to_string(this->sensor_count) << std::endl;
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







