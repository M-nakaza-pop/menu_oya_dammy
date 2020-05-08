#ifndef __DEBUG_H__
#define __DEBUG_H__

#define DEBUG
#ifdef DEBUG
    #define BeginDebugPrint()    Serial.begin( 19200 )
    #define DebugPrint( message )\
        {\
            char __buff__[ 512 ];\
            sprintf( __buff__\
                   , "%s (Func:%s, File:%s, Line:%d)"\
                   , message\
                   , __func__\
                   , __FILE__\
                   , __LINE__ );\
            Serial.println( __buff__ );\
            Serial.flush();\
        }
#else
    #define BeginDebugPrint()
    #define DebugPrint( message )
#endif // DEBUG
#endif // __DEBUG_H__				

