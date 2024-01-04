/**
 * Created by Alexey Presnyakov 2024
 * Based on halui and emcrsh code
 */
#ifndef LINUXCNC_EMCAPI_H
#define LINUXCNC_EMCAPI_H


class EmcApi {
public:
    EmcApi() { };

    int init(const char* iniFilePath);

    void update();
};


#endif //LINUXCNC_EMCAPI_H
