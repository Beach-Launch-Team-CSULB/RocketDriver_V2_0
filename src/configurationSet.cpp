#include "configurationSet.h"
#include <unordered_map>
//#include "CANRead.h"

//still not sure exactly the best way to do this
//is there a way to map directly to the set functions of my class objects?
//That would have to be with pointers specifically to my set functions somehow
//This should work, but even if it does I need nested maps. Inner map is the key to find the right function pointer for an object, outer map is to grab the right map for an object.



//the shittier version is to just make switch cases for each class based on the settingID
//would need to map the objectIDs to the right class, and have a stupid switch function for the class manually coded

std::unordered_map<uint16_t, configSet> configSetmap
{
// map key = ALARA address
{1, configSet {0, 0, 0}},
{2, configSet {0, 0, 0}},
{3, configSet {0, 0, 0}},
{4, configSet {0, 0, 0}},
};


void lookupALARASNmap(configMSG MSGin)
{
    
    
    //thisALARA = configSetmap[ALARANodeIDIn];
};
