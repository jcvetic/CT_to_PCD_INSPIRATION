#include "visualizer.h"

int main() {
    // std::string folderPath = "/home/jcvetic/Patient_registration_cveta/Patient_registration/CT_dicom/Banovic_CT+";
    // std::string folderPath = "/home/jcvetic/Patient_registration_cveta/Patient_registration/CT_dicom/Pehar_CT";
    // std::string folderPath = "/home/jcvetic/Patient_registration_cveta/Patient_registration/CT_dicom/Ridanec_CT";
    // std::string folderPath = "/home/jcvetic/Patient_registration_cveta/Patient_registration/CT_dicom/Cvitan_CT"; // cijeli u banani - puno okolnog suma
    // std::string folderPath = "/home/jcvetic/Patient_registration_cveta/Patient_registration/CT_dicom/Jurak_CT";
    // std::string folderPath = "/home/jcvetic/Patient_registration_cveta/Patient_registration/CT_dicom/Subotic_CT";

    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/slobodnoCT/Vukovic_CT";
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/slobodnoCT/Brankovic_CT";
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/slobodnoCT/Serdarusic_CT";
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/slobodnoCT/Zugaj_CT"; // dobar primjer
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/slobodnoCT/Puhalovic_CT";
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/slobodnoCT/Boros_CT";
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/slobodnoCT/Capin_CT";
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/slobodnoCT/Herceg_CT";
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/slobodnoCT/Ledinski_CT";
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/slobodnoCT/Palinic_CT";

    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/dicombaza/Agbaba_CT";
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/dicombaza/Dedus_CT";
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/dicombaza/Krhen_CT";
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/dicombaza/Maras_CT"; // premalo CT slika=? - ne valja
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/dicombaza/Milovanovic_CT";
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/dicombaza/Rebernik_CT"; // ista stvar kao Maras!
    // std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/dicombaza/Soplanda_CT";

    std::string folderPath = "/home/jcvetic/Visualize_pointclouds/data/CQ500/thin/patient6";
    visualizer vis1(folderPath);
    vis1.run();
    return 0;
}
