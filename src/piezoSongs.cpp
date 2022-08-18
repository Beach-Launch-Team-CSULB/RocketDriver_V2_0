// NB: ALL NOTES DEFINED WITH STANDARD ENGLISH NAMES
// Frequencies for equal-tempered scale, A4 = 440 Hz, C4 is middle C
// Taken from https://pages.mtu.edu/~suits/notefreqs.html
#include "piezoSongs.h"
#include "pitches.h"


//Generate the songs with vectors and functions that assemble them from notes

//std::vector<buzzerTone> songTheMandalorianTheme()
//{

//}


std::vector<buzzerTone> songTheImperialMarch()
{
  //
    // Default rest tone of length zero
    buzzerTone restTone = {0,0,0};
    std::vector<buzzerTone> songVector{};
 
    songVector.push_back({100,NOTE_A3,Q});
    songVector.push_back({100,NOTE_A3,Q});
    songVector.push_back({100,NOTE_A3,Q});
    songVector.push_back({100,NOTE_F3,E+S});
    songVector.push_back({100,NOTE_C4,S});
    
    songVector.push_back({100,NOTE_A3,Q});
    songVector.push_back({100,NOTE_F3,E+S});
    songVector.push_back({100,NOTE_C4,S});
    songVector.push_back({100,NOTE_A3,H});
    songVector.push_back({100,NOTE_C4,S});

    songVector.push_back({100,NOTE_E4,Q});
    songVector.push_back({100,NOTE_E4,Q});
    songVector.push_back({100,NOTE_E4,Q});
    songVector.push_back({100,NOTE_F4,E+S});
    songVector.push_back({100,NOTE_C4,S});

    songVector.push_back({100,NOTE_Ab3,Q});
    songVector.push_back({100,NOTE_F3,E+S});
    songVector.push_back({100,NOTE_C4,S});
    songVector.push_back({100,NOTE_A3,H});

    songVector.push_back({100,NOTE_A4,Q});
    songVector.push_back({100,NOTE_A3,E+S});
    songVector.push_back({100,NOTE_A3,S});
    songVector.push_back({100,NOTE_A4,Q});
    songVector.push_back({100,NOTE_Ab4,E+S});
    songVector.push_back({100,NOTE_G4,S});
    
    songVector.push_back({100,NOTE_Gb4,S});
    songVector.push_back({100,NOTE_E4,S});
    songVector.push_back({100,NOTE_F4,E});
    songVector.push_back({100,NOTE_REST,E});
    songVector.push_back({100,NOTE_Bb3,E});
    songVector.push_back({100,NOTE_Eb4,Q});
    songVector.push_back({100,NOTE_D4,E+S});
    songVector.push_back({100,NOTE_Db4,S});

    songVector.push_back({100,NOTE_C4,S});
    songVector.push_back({100,NOTE_B3,S});
    songVector.push_back({100,NOTE_C4,E});
    songVector.push_back({100,NOTE_REST,E});
    songVector.push_back({100,NOTE_F3,E});
    songVector.push_back({100,NOTE_Ab3,Q});
    songVector.push_back({100,NOTE_F3,E+S});
    songVector.push_back({100,NOTE_A3,S});

    songVector.push_back({100,NOTE_C4,Q});
    songVector.push_back({100,NOTE_A3,E+S});
    songVector.push_back({100,NOTE_C4,S});
    songVector.push_back({100,NOTE_E4,H});

    songVector.push_back({100,NOTE_A4,Q});
    songVector.push_back({100,NOTE_A3,E+S});
    songVector.push_back({100,NOTE_A3,S});
    songVector.push_back({100,NOTE_A4,Q});
    songVector.push_back({100,NOTE_Ab4,E+S});
    songVector.push_back({100,NOTE_G4,S});

    songVector.push_back({100,NOTE_Gb4,S});
    songVector.push_back({100,NOTE_E4,S});
    songVector.push_back({100,NOTE_F4,E});
    songVector.push_back({100,NOTE_REST,E});
    songVector.push_back({100,NOTE_Bb3,E});
    songVector.push_back({100,NOTE_Eb4,Q});
    songVector.push_back({100,NOTE_D4,E+S});
    songVector.push_back({100,NOTE_Db4,S});

    songVector.push_back({100,NOTE_C4,S});
    songVector.push_back({100,NOTE_B3,S});
    songVector.push_back({100,NOTE_C4,E});
    songVector.push_back({100,NOTE_REST,E});
    songVector.push_back({100,NOTE_F3,E});
    songVector.push_back({100,NOTE_Ab3,Q});
    songVector.push_back({100,NOTE_F3,E+S});
    songVector.push_back({100,NOTE_C4,S});

    songVector.push_back({100,NOTE_A3,Q});
    songVector.push_back({100,NOTE_F3,E+S});
    songVector.push_back({100,NOTE_C4,S});
    songVector.push_back({100,NOTE_A3,H});

    songVector.push_back({100,NOTE_REST,2*H});
    
    return songVector;
}