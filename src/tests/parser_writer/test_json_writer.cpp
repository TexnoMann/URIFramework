#include <jsoncpp/json/json.h>
#include <jsoncpp/json/writer.h>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char const* argv[]){

  std::ofstream op_file_id;
    op_file_id.open("file.txt");

    Json::Value value_obj;

    Json::Value vec(Json::arrayValue);
    vec.append(Json::Value(1));
    vec.append(Json::Value(2));
    vec.append(Json::Value(3));

    value_obj["test"]["b1"]["name"] = "Lesha";
    value_obj["test"]["b2"]["code"] = 228337;
    value_obj["test"]["b1"]["name"] = "MOLODEC";
    value_obj["test"]["b2"]["code"]=vec;

    Json::StyledWriter styledWriter;
    op_file_id << styledWriter.write(value_obj);

    op_file_id.close();
}
