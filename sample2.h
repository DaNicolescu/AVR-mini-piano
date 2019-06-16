#ifndef SAMPLE2_H
#define SAMPLE2_H

#include <avr/pgmspace.h>

int sample2_size = 7980;

const unsigned char sample2[7980] PROGMEM = {
    128,127,128,129,130,129,125,123,102,101,102,110,117,140,145,143,151,137,131,128,153,174,182,149,115,124,140,123,115,118,98,97
    ,104,97,127,138,114,116,116,123,132,151,154,174,165,157,155,130,106,114,120,120,152,122,75,98,128,121,140,120,115,125,112,95
    ,115,119,91,98,91,94,109,135,158,188,178,161,149,137,121,122,116,104,137,132,93,110,138,148,171,141,139,142,114,93,116,130
    ,96,110,101,96,107,124,158,178,182,164,161,146,122,121,117,113,141,146,101,99,103,108,130,104,113,117,99,86,120,157,134,150
    ,120,106,107,116,152,169,179,145,139,128,99,107,98,96,135,153,121,108,104,113,136,117,129,136,115,95,131,156,127,138,111,99
    ,88,87,124,147,173,158,161,155,122,133,116,106,138,144,117,97,86,96,116,104,111,132,117,96,134,160,153,159,140,127,115,110
    ,145,164,185,170,159,148,104,115,91,78,117,139,130,112,103,114,130,119,107,133,116,89,131,152,156,148,140,125,112,106,132,164
    ,186,183,184,177,135,138,104,69,98,115,113,92,75,86,101,103,88,133,122,100,142,161,177,164,161,147,128,113,118,153,176,185
    ,195,181,145,141,118,79,101,111,115,90,68,79,95,99,72,121,117,102,136,160,184,172,172,161,135,116,107,145,167,180,199,186
    ,163,158,139,91,101,108,120,106,83,83,104,106,80,126,127,113,131,158,187,182,183,176,151,129,106,140,156,168,194,180,162,162
    ,146,104,103,115,126,115,89,76,101,93,74,106,113,98,100,129,160,161,163,160,138,113,83,114,134,152,191,177,161,163,148,115
    ,98,110,115,119,95,78,111,95,83,101,118,111,104,131,158,165,162,159,145,118,84,108,127,140,190,182,167,173,167,140,108,117
    ,115,135,105,82,114,94,87,93,116,112,96,117,140,156,152,149,147,120,85,103,120,132,186,187,173,175,170,142,106,104,99,126
    ,97,76,111,100,94,93,116,114,92,106,121,148,150,149,155,127,89,95,107,114,163,175,162,164,165,145,117,106,105,137,114,88
    ,114,104,94,86,108,112,90,99,110,143,148,152,170,145,109,108,112,116,156,184,173,175,176,152,130,101,98,127,112,85,103,104
    ,93,87,109,118,101,103,106,134,144,144,167,142,106,99,95,101,133,172,161,167,173,154,140,99,97,124,119,94,100,112,96,89
    ,104,113,105,99,100,127,144,140,170,151,119,109,101,106,130,179,171,182,189,173,166,115,103,127,128,106,99,117,99,92,104,112
    ,112,102,99,123,141,137,168,157,126,111,96,94,118,167,168,181,191,181,182,129,107,126,129,112,97,118,98,89,95,98,105,94
    ,90,111,128,129,161,158,132,119,104,91,109,153,162,179,188,184,195,143,109,120,125,111,88,109,93,83,89,88,105,96,89,112
    ,126,134,164,172,147,136,123,96,110,146,159,175,184,186,202,156,113,114,123,112,92,112,104,96,100,97,113,101,87,109,114,121
    ,147,167,148,140,133,98,112,139,155,178,181,190,206,173,124,112,121,106,90,100,96,88,91,88,106,96,79,102,103,112,135,163
    ,150,144,146,102,111,127,144,173,170,189,206,192,139,119,128,109,98,100,101,93,91,87,106,97,80,103,103,110,132,165,156,151
    ,155,106,109,114,133,165,162,185,207,210,156,129,135,114,103,99,98,95,87,81,103,95,76,93,96,93,113,150,147,150,156,112
    ,109,106,126,162,163,182,206,220,172,138,143,122,111,106,100,108,92,86,105,99,79,88,97,85,106,145,148,159,164,130,120,110
    ,124,157,163,173,200,224,188,147,148,127,115,107,95,109,89,83,103,102,83,86,103,82,98,134,140,160,165,142,125,109,115,145
    ,159,163,193,225,203,161,154,132,118,107,90,106,84,78,98,103,85,84,110,86,99,130,138,168,169,158,137,115,112,137,158,155
    ,185,224,213,176,164,142,128,114,97,113,91,79,95,106,88,84,112,86,98,125,137,171,172,169,151,123,110,127,153,145,169,213
    ,213,188,168,150,139,119,102,114,101,83,93,107,86,78,104,81,89,107,115,151,159,164,153,127,110,121,151,145,162,207,214,200
    ,175,155,143,117,100,105,100,76,85,108,89,85,103,87,92,100,108,140,152,159,156,131,106,111,143,140,152,201,214,213,186,166
    ,155,120,101,101,104,78,81,110,89,87,97,86,89,92,102,132,151,159,164,144,113,108,137,134,138,180,199,210,188,168,163,130
    ,107,101,107,80,76,106,82,83,87,80,85,82,91,115,140,150,162,152,115,104,130,128,129,165,193,215,198,178,177,146,117,105
    ,112,81,71,103,83,87,84,79,84,76,83,101,130,143,159,161,122,105,124,124,124,148,181,210,200,178,176,150,115,99,109,78
    ,68,94,79,88,83,79,85,77,76,88,115,131,148,163,126,103,117,118,121,133,168,204,203,186,183,169,127,108,118,85,75,91
    ,84,90,84,77,81,76,72,81,106,123,140,166,135,109,117,117,122,124,156,197,207,196,190,189,143,122,129,95,85,89,90,93
    ,88,82,87,85,76,82,103,121,138,172,147,118,118,116,126,119,147,189,206,201,195,205,157,134,138,104,91,86,90,91,86,80
    ,83,83,73,76,96,111,130,169,157,132,124,120,131,117,137,176,201,199,191,207,162,140,142,111,99,90,94,95,91,87,87,91
    ,80,79,95,104,124,165,163,140,124,120,129,115,127,162,197,198,196,216,179,154,151,122,105,91,92,97,92,90,86,93,83,78
    ,95,97,118,158,168,154,131,128,134,121,123,152,196,196,199,220,191,164,154,129,110,94,92,98,94,93,88,95,84,76,93,89
    ,108,146,164,160,131,126,127,120,113,135,186,186,195,217,199,176,162,139,117,96,91,97,95,94,87,98,87,76,91,81,96,129
    ,156,165,134,128,125,123,108,122,174,177,190,212,202,184,167,148,125,100,91,92,93,90,84,97,87,74,87,76,84,112,147,165
    ,139,132,127,131,109,112,165,171,185,209,206,194,175,162,139,111,98,93,100,93,87,99,88,76,87,78,78,97,137,163,148,138
    ,132,139,114,109,155,163,177,202,207,201,181,169,146,116,100,90,100,91,85,101,90,81,87,82,77,85,124,154,152,136,127,139
    ,113,102,142,155,171,197,207,205,188,181,161,132,110,94,104,94,87,104,94,87,88,87,78,77,114,146,159,140,130,145,121,104
    ,133,149,164,187,205,207,195,189,173,146,119,96,106,96,87,106,99,96,92,96,86,73,101,133,160,143,131,150,127,107,124,142
    ,156,177,199,206,199,192,179,158,128,97,105,94,85,101,98,95,87,95,86,70,88,119,158,146,133,152,132,111,117,133,146,162
    ,187,198,196,194,183,168,136,102,108,96,86,96,98,96,86,96,89,74,77,104,150,146,131,149,135,112,110,124,139,152,179,194
    ,196,197,186,181,149,111,111,97,88,90,98,95,85,94,89,78,68,89,140,145,131,148,141,120,113,121,134,141,168,186,193,197
    ,187,191,160,121,115,100,92,85,96,92,83,90,88,83,61,72,124,136,126,142,142,123,110,115,127,134,160,180,193,198,187,197
    ,170,131,117,102,95,83,95,92,85,89,87,90,61,61,110,132,128,140,145,131,113,114,122,128,151,170,190,196,187,201,181,146
    ,125,109,98,83,94,92,85,85,83,93,64,51,96,123,126,136,146,137,117,115,117,123,142,158,185,192,188,204,191,161,133,117
    ,103,87,94,92,89,84,82,97,71,46,81,111,122,132,146,145,123,120,117,121,137,150,182,189,187,203,200,178,147,130,112,95
    ,97,95,97,89,85,105,85,52,73,102,118,127,142,147,127,124,116,121,133,140,176,186,187,202,206,190,156,136,114,96,93,90
    ,97,88,84,107,95,59,66,91,110,120,137,147,131,126,113,117,124,127,165,177,182,194,205,198,166,147,122,103,96,90,102,91
    ,81,105,101,68,62,84,105,115,133,146,135,130,115,119,121,121,156,172,178,189,206,207,182,162,134,115,105,94,108,95,81,104
    ,108,81,63,79,102,111,132,146,142,138,119,122,121,120,148,168,177,182,202,209,192,174,143,124,109,94,107,97,80,99,109,91
    ,67,74,94,102,125,137,140,138,119,123,118,116,137,160,172,176,197,208,201,185,153,133,114,95,108,100,80,94,109,100,73,72
    ,88,94,119,130,142,142,122,126,118,115,128,153,169,171,193,205,206,194,163,143,121,98,109,106,85,92,109,109,80,71,82,88
    ,112,123,141,144,127,129,120,116,121,146,164,167,188,201,211,206,177,158,132,104,110,111,92,90,107,115,88,76,79,83,105,115
    ,138,144,131,131,123,116,115,136,156,160,179,192,209,211,185,169,142,109,111,113,96,85,101,115,93,80,74,78,95,105,131,142
    ,134,133,126,116,110,128,148,155,171,181,202,212,191,179,152,115,111,112,100,81,96,114,97,87,72,75,86,95,122,136,134,131
    ,127,117,107,120,139,150,164,172,196,212,196,188,165,125,114,112,106,80,89,110,102,96,76,77,81,87,112,129,133,130,129,120
    ,106,115,129,144,158,162,186,208,200,198,178,138,118,114,114,84,87,105,104,99,77,76,76,78,100,120,130,129,131,124,107,112
    ,120,137,151,153,176,201,200,202,188,153,125,119,122,91,86,98,105,103,83,78,76,75,94,116,130,129,132,127,108,110,114,132
    ,149,149,169,194,198,206,198,169,133,125,127,99,87,93,106,107,90,82,79,75,88,110,126,127,133,131,114,110,109,124,145,142
    ,161,186,195,206,205,186,144,133,134,110,93,91,105,109,97,88,82,75,81,103,122,126,134,135,120,112,106,119,142,138,153,176
    ,190,203,209,199,155,138,137,119,101,91,103,110,100,93,84,76,74,94,113,120,131,135,125,116,104,112,135,133,146,166,183,197
    ,209,209,168,145,141,126,108,88,98,108,102,95,86,78,68,85,105,114,127,132,126,118,101,107,128,130,141,156,177,188,206,216
    ,181,152,142,130,114,89,93,104,103,97,90,82,68,79,97,106,122,130,128,126,103,105,123,130,137,149,171,180,202,220,194,163
    ,148,139,125,97,94,103,106,101,96,89,71,73,90,99,117,126,129,131,107,104,116,128,131,142,163,170,193,218,205,175,153,145
    ,132,104,93,99,105,100,98,92,74,70,83,90,110,120,128,135,111,103,109,123,125,133,155,159,180,210,209,187,161,153,140,115
    ,95,96,104,100,100,97,80,72,80,85,106,116,127,137,118,105,106,121,123,129,150,153,171,203,215,200,171,161,150,128,103,98
    ,105,103,106,107,90,81,83,85,104,112,125,139,129,112,106,121,124,128,148,150,164,193,215,209,182,168,159,139,112,101,107,104
    ,106,111,95,87,82,83,99,106,120,136,134,116,105,118,120,124,142,146,155,180,209,213,192,174,166,149,122,104,107,102,104,112
    ,98,92,82,81,95,102,115,132,139,121,104,113,115,119,135,141,146,166,198,213,199,179,172,160,134,111,110,102,104,113,102,97
    ,85,81,92,98,108,124,139,126,107,113,113,116,130,139,142,156,187,210,206,185,176,168,143,117,113,103,104,112,105,102,89,81
    ,87,92,101,115,136,131,110,111,108,111,123,135,138,146,173,203,208,191,179,174,151,125,116,102,102,110,107,105,93,81,84,89
    ,95,105,131,132,113,110,106,107,117,130,133,138,160,192,207,196,182,180,159,133,120,103,101,106,106,107,97,83,81,85,88,95
    ,123,131,118,113,107,105,112,125,130,131,146,177,202,199,184,185,167,142,127,107,103,104,105,108,100,87,81,85,85,87,114,130
    ,123,115,109,103,109,121,129,129,137,165,196,201,188,191,176,153,137,114,106,104,105,110,105,94,83,87,85,81,105,126,126,119
    ,112,104,106,118,128,126,129,151,185,199,190,194,183,163,147,121,110,104,104,110,108,100,86,90,87,79,96,119,125,121,115,105
    ,103,113,125,127,125,139,175,195,192,197,188,172,157,130,117,107,104,109,111,105,90,93,90,80,90,113,124,125,119,109,103,109
    ,121,126,123,128,163,188,192,199,194,182,168,143,126,113,107,108,113,110,94,96,95,83,86,108,122,128,124,115,103,107,118,128
    ,125,123,154,179,188,199,196,189,176,154,135,119,111,107,115,113,98,96,98,85,82,100,117,127,127,120,105,103,112,125,124,116
    ,141,167,181,195,194,193,183,164,144,126,116,107,114,114,100,96,100,88,80,91,109,123,127,123,108,102,105,122,124,112,131,155
    ,173,190,193,195,187,172,151,133,120,108,113,117,105,98,103,93,81,86,101,117,126,125,113,103,101,118,125,112,123,145,166,186
    ,192,197,192,181,161,142,127,111,112,119,109,101,105,99,85,83,94,109,124,126,117,104,95,112,124,112,117,134,156,176,188,195
    ,194,187,168,149,132,113,109,118,111,102,104,102,89,81,88,101,118,125,121,109,93,107,121,112,112,123,145,165,182,191,196,191
    ,176,158,141,119,109,117,112,103,103,104,93,83,84,93,111,122,124,115,94,102,117,114,111,116,137,156,176,187,195,196,184,167
    ,152,127,113,118,115,106,104,107,98,87,83,87,104,117,125,123,98,100,113,116,112,112,128,147,167,180,191,196,188,174,162,136
    ,116,116,117,108,105,108,103,93,84,82,98,110,124,127,102,97,107,114,112,110,121,138,159,174,186,197,192,179,170,146,121,116
    ,118,110,106,108,106,98,87,81,93,103,120,129,108,97,102,112,112,109,114,129,150,166,180,196,195,185,179,158,130,120,120,112
    ,106,109,107,103,92,82,91,97,116,131,116,100,99,108,111,109,110,122,141,158,172,191,196,188,186,170,141,124,122,116,108,109
    ,108,108,96,83,88,89,109,129,123,105,98,106,111,110,108,115,133,149,163,183,193,189,190,181,154,133,127,120,112,111,108,112
    ,102,88,88,85,102,125,128,112,101,104,110,112,109,110,126,142,155,175,190,189,192,188,165,140,131,123,115,112,108,115,106,94
    ,91,83,93,117,129,117,103,102,107,111,109,107,120,136,147,167,185,188,192,193,175,149,136,126,119,114,109,117,110,100,95,85
    ,87,110,128,122,108,101,103,109,110,105,115,130,139,160,180,187,192,197,185,159,144,129,124,116,110,118,114,107,100,89,83,102
    ,123,126,114,104,103,109,111,105,110,124,132,151,172,183,190,199,193,170,153,134,130,120,111,117,115,109,105,93,82,93,116,126
    ,119,108,102,107,111,104,106,119,125,141,164,177,184,198,197,179,162,140,135,123,111,116,116,111,108,98,83,85,107,123,123,113
    ,103,105,110,105,104,114,119,132,154,170,178,195,198,188,171,147,139,127,114,115,116,112,112,105,88,82,99,119,124,117,104,103
    ,109,105,103,111,116,125,147,164,172,191,197,194,179,155,145,133,117,115,116,113,114,110,94,82,93,113,124,121,107,103,109,105
    ,103,108,113,116,137,156,165,185,194,198,187,164,152,140,123,116,116,113,114,114,100,83,85,104,119,125,111,102,107,105,102,104
    ,111,110,129,147,158,176,188,198,193,173,159,147,128,119,117,113,114,117,106,88,82,96,114,127,115,104,106,105,101,103,108,106
    ,121,137,150,168,180,195,196,179,166,154,136,123,119,114,114,118,113,95,83,90,107,125,119,107,106,106,100,102,106,103,114,129
    ,144,160,173,190,198,184,171,160,142,127,121,115,113,118,118,102,87,84,98,119,120,110,105,106,98,101,103,100,109,120,136,152
    ,164,183,196,189,177,167,149,132,124,116,112,116,121,108,93,83,92,113,122,113,107,108,98,100,102,99,105,114,129,145,156,176
    ,194,193,182,173,155,138,128,119,112,115,122,113,100,84,86,104,120,115,109,110,100,101,102,99,102,107,121,137,148,165,188,193
    ,185,179,163,146,132,123,113,113,121,118,106,88,82,96,116,115,111,111,101,101,101,99,100,102,114,130,141,155,180,191,188,182
    ,169,153,138,128,114,112,118,121,113,96,82,88,110,113,112,113,104,102,101,99,100,100,108,124,134,146,171,186,189,186,177,161
    ,145,134,118,112,116,121,118,104,85,84,105,113,115,115,108,103,101,100,100,99,103,118,129,138,162,181,190,189,183,168,153,141
    ,125,115,114,122,121,114,92,84,100,110,116,117,112,107,103,101,101,99,100,114,124,131,152,173,187,190,189,177,163,150,134,119
    ,115,121,124,122,100,85,95,106,115,120,116,111,104,102,102,101,99,111,121,127,143,164,182,189,192,183,171,159,143,125,119,120
    ,125,129,110,90,93,102,114,121,120,115,109,104,104,102,98,107,117,123,135,155,175,187,193,189,178,167,153,133,124,120,125,133
    ,119,98,94,99,110,120,121,118,112,104,105,103,97,104,113,120,128,147,167,182,192,191,182,173,160,140,129,120,123,135,127,107
    ,96,97,106,117,121,119,116,106,106,104,97,100,109,116,122,138,157,175,188,192,185,178,167,147,134,121,119,133,132,114,99,95
    ,101,113,120,121,119,108,106,104,97,97,104,111,115,129,147,166,182,192,187,182,173,154,140,124,117,129,134,121,103,94,95,107
    ,115,119,120,110,108,106,100,96,101,108,111,121,137,155,174,188,186,184,177,161,148,129,116,124,133,127,110,98,94,103,110,117
    ,121,112,108,107,102,95,98,104,108,115,130,146,166,183,185,185,181,168,155,136,118,120,131,130,117,102,94,100,105,114,120,114
    ,110,108,104,96,96,102,104,109,122,136,156,176,182,186,184,173,163,145,123,117,127,131,124,108,95,98,101,112,119,117,112,110
    ,106,97,94,99,102,105,116,127,146,169,179,185,186,178,170,154,131,118,125,130,129,115,100,97,98,108,117,119,114,112,108,99
    ,94,98,100,102,111,119,136,159,173,183,186,181,176,165,140,121,123,128,132,120,105,99,96,103,115,120,116,114,111,102,94,97
    ,98,99,107,113,127,149,164,178,184,182,179,173,150,128,123,126,134,126,112,101,95,98,110,117,117,114,113,105,95,96,97,97
    ,104,107,118,138,156,172,181,182,180,179,159,136,124,124,133,130,118,106,97,96,105,115,117,115,115,109,97,95,95,95,101,103
    ,111,129,147,164,178,181,180,183,168,145,127,122,130,132,124,112,100,95,100,112,117,115,116,112,100,96,94,93,98,99,105,119
    ,138,156,174,179,179,184,175,155,132,121,127,133,128,118,104,96,97,108,115,114,117,115,103,97,94,92,97,97,101,112,129,147
    ,168,176,178,184,180,165,141,124,126,132,131,124,111,99,95,104,113,114,118,119,109,103,96,93,96,97,99,107,121,137,161,173
    ,176,183,185,175,153,130,126,131,133,129,117,105,95,101,111,113,118,122,113,107,100,95,97,98,98,104,115,128,153,168,174,182
    ,186,183,164,139,129,130,134,132,124,112,98,100,109,111,118,123,118,112,103,98,97,100,97,102,110,121,145,163,172,180,186,188
    ,175,150,133,130,134,135,131,120,103,100,109,111,118,124,122,117,108,100,98,100,97,101,107,115,136,156,168,177,184,191,184,162
    ,140,133,135,136,135,128,110,101,107,108,115,123,124,121,113,103,98,101,97,100,104,109,127,148,163,173,181,191,190,173,149,136
    ,134,135,136,134,116,104,106,107,113,122,126,124,119,107,99,100,96,98,101,103,118,139,156,168,176,187,193,182,159,141,135,134
    ,136,138,122,108,105,105,109,118,124,125,123,110,102,100,97,98,100,99,110,128,147,160,169,181,191,188,168,147,138,134,136,141
    ,128,113,106,104,106,114,121,124,125,114,105,100,96,97,98,97,103,118,138,153,162,174,187,191,177,154,141,134,133,142,133,119
    ,109,105,103,110,118,122,127,119,109,102,96,96,98,95,98,109,128,146,156,167,182,192,185,162,147,134,131,140,137,125,113,106
    ,102,107,114,120,127,122,112,105,96,96,97,95,95,102,119,138,150,160,174,189,189,169,154,138,130,137,139,130,118,110,102,105
    ,110,116,125,123,116,109,98,96,97,95,93,96,110,129,143,154,166,184,191,177,162,144,131,135,139,133,123,113,103,104,107,112
    ,123,124,119,113,102,97,97,96,93,93,102,120,136,148,157,177,189,182,170,151,134,133,139,135,128,118,106,103,104,108,119,123
    ,122,117,105,98,98,97,94,90,96,111,128,142,148,169,185,184,177,159,139,132,137,136,131,123,109,105,104,105,116,123,124,121
    ,110,100,98,97,95,89,92,104,120,136,141,160,178,184,182,167,146,134,136,136,134,128,114,108,103,102,112,119,124,124,115,104
    ,99,97,97,90,90,96,113,129,134,151,170,181,184,176,155,138,136,136,136,132,119,112,105,101,108,116,123,127,120,109,101,98
    ,98,91,90,91,107,123,129,143,162,176,184,182,164,145,138,136,138,136,124,117,108,102,106,113,121,128,124,115,105,100,100,95
    ,92,88,100,117,124,136,153,169,182,186,173,153,142,136,139,140,130,123,113,104,106,110,118,128,127,121,108,103,101,97,95,87
    ,95,112,121,130,146,162,177,188,180,161,148,138,140,142,135,128,119,108,107,108,115,126,129,126,114,107,102,100,98,88,91,106
    ,116,125,138,155,170,186,185,170,155,141,139,143,138,133,124,112,108,106,112,122,130,130,119,112,104,102,101,90,88,100,111,120
    ,132,147,162,182,187,177,162,146,140,143,141,137,130,117,111,106,110,118,128,132,123,116,107,104,105,94,88,96,107,115,126,140
    ,154,176,187,183,171,152,142,143,141,140,133,122,114,107,109,114,126,133,127,121,111,106,107,98,89,92,102,110,121,133,145,167
    ,183,185,177,159,146,144,141,143,137,127,118,109,108,109,121,131,129,125,114,108,109,102,91,89,98,104,115,126,136,157,176,184
    ,183,166,151,145,141,143,139,132,122,112,109,106,117,129,131,130,119,110,110,105,94,89,95,100,111,121,129,148,168,181,186,173
    ,158,148,142,144,141,136,126,117,111,105,113,125,130,133,124,114,111,109,98,90,92,96,106,116,122,138,158,175,187,178,165,152
    ,144,145,143,140,131,122,114,105,109,120,128,134,128,117,112,112,101,92,91,92,102,112,116,130,148,166,183,181,171,157,146,144
    ,143,142,134,126,119,107,107,114,124,133,131,121,115,114,105,96,91,89,97,107,111,122,137,156,177,181,176,162,150,145,143,144
    ,137,130,123,110,106,110,119,130,133,124,116,116,108,98,93,88,93,103,106,116,128,146,169,178,179,167,154,145,142,144,139,134
    ,128,113,107,106,113,125,133,127,118,117,111,102,95,87,89,98,102,111,119,136,158,173,180,172,159,147,142,144,139,135,131,118
    ,109,105,109,119,131,129,120,118,113,105,99,88,87,93,98,107,112,126,147,165,178,175,165,151,143,144,140,138,134,123,112,105
    ,105,113,128,130,123,120,116,109,103,91,88,91,95,104,106,118,137,157,174,177,171,156,146,145,141,139,137,129,116,108,104,108
    ,124,131,126,123,119,113,108,95,90,89,92,101,103,113,128,148,168,176,176,162,150,148,143,141,139,134,122,113,105,105,119,130
    ,128,126,122,117,113,100,94,90,91,99,101,108,120,139,160,173,179,168,156,151,145,143,142,138,128,119,109,103,115,128,129,130
    ,124,121,118,106,98,91,91,97,100,105,114,131,151,168,180,173,162,155,149,144,144,142,134,125,114,104,111,125,129,133,127,125
    ,122,112,103,94,91,95,98,103,108,123,142,161,178,177,167,160,153,146,145,145,139,130,121,106,108,120,128,134,130,127,126,117
    ,109,98,93,94,97,102,104,116,132,151,172,178,171,164,157,149,147,147,142,136,128,111,107,116,125,134,130,129,128,121,114,101
    ,95,93,95,100,100,110,123,141,164,176,174,167,160,151,147,148,143,139,133,116,108,111,120,132,131,130,129,123,118,106,97,93
    ,94,98,98,105,115,130,154,171,174,169,163,154,147,148,144,142,138,122,111,108,115,128,130,130,129,126,122,110,101,94,93,96
    ,96,101,108,120,143,164,172,171,166,158,149,149,144,143,142,128,116,107,111,124,129,131,130,128,125,115,105,95,93,95,95,99
    ,104,112,132,155,169,171,169,161,151,150,145,144,144,133,121,108,108,119,125,130,129,129,127,120,111,99,95,95,94,97,100,105
    ,122,144,162,169,171,164,154,151,145,144,145,138,127,111,107,115,122,129,129,129,128,124,116,103,96,95,93,95,98,101,112,134
    ,154,165,171,167,157,153,146,144,146,141,134,116,107,111,118,127,128,130,129,127,121,108,99,96,93,94,96,98,104,123,145,158
    ,170,169,160,156,148,144,145,143,139,122,110,108,114,124,127,129,130,129,125,113,102,97,93,93,95,96,98,113,135,151,166,169
    ,163,159,150,146,144,144,144,129,116,108,111,120,126,129,130,130,129,118,107,100,94,93,94,96,94,105,125,141,161,168,165,162
    ,153,148,144,144,146,135,122,110,109,116,124,128,129,130,131,123,112,104,96,93,94,96,92,99,116,132,154,164,166,165,156,151
    ,144,145,148,141,129,114,109,113,120,127,128,130,133,127,118,108,100,94,94,97,92,95,108,123,145,159,165,166,159,155,146,144
    ,148,145,136,120,111,111,117,126,128,129,133,131,123,112,104,96,94,97,92,93,102,114,136,152,163,166,162,158,149,145,148,148
    ,142,126,114,111,114,123,126,128,133,133,128,118,110,99,95,99,93,92,97,106,127,144,159,165,164,161,152,146,147,149,146,133
    ,119,112,112,121,125,128,132,135,132,123,116,104,97,100,94,93,95,101,118,136,154,163,165,164,156,149,147,149,150,140,126,116
    ,112,119,124,127,131,135,135,128,122,109,100,102,96,95,94,97,111,127,147,159,165,167,160,153,148,149,153,145,134,120,113,117
    ,122,126,129,135,136,131,128,115,104,103,98,97,94,94,104,118,139,153,162,167,163,156,149,149,153,149,140,126,116,115,120,125
    ,127,134,137,133,132,120,108,105,99,99,94,93,99,110,130,146,158,167,165,160,151,149,153,152,146,131,119,115,118,123,125,133
    ,136,135,136,126,113,107,101,100,95,93,95,103,121,138,152,163,166,163,154,149,151,153,150,137,124,116,116,121,122,130,135,134
    ,138,130,119,110,103,102,97,94,93,97,112,129,145,158,164,165,157,150,150,152,152,143,130,118,116,119,119,127,132,133,139,134
    ,124,114,106,103,98,95,92,93,105,120,137,151,161,166,159,152,149,151,153,147,135,122,116,118,117,124,130,131,138,136,129,118
    ,109,105,99,97,92,90,98,111,128,144,157,165,161,155,149,149,153,150,141,126,118,118,115,122,127,129,135,137,133,123,112,106
    ,101,99,93,89,94,103,120,135,150,162,162,157,150,148,151,151,147,130,121,119,113,119,124,127,132,136,136,127,117,109,102,100
    ,95,89,90,97,111,126,142,157,162,159,152,148,150,151,151,135,125,120,113,117,121,124,129,135,137,131,121,113,104,102,97,91
    ,88,92,104,117,133,150,159,161,154,149,148,150,153,140,129,122,114,115,118,122,126,132,137,134,126,116,107,104,99,93,88,89
    ,98,110,124,142,155,161,156,151,147,149,155,144,134,126,116,114,116,120,124,129,137,136,130,121,111,107,101,96,90,87,94,103
    ,117,133,150,159,157,154,147,148,155,149,139,131,120,116,115,118,122,127,135,137,134,126,116,109,104,99,92,87,91,98,110,125
    ,143,156,158,157,149,147,154,151,144,135,124,118,115,117,120,124,132,136,137,131,120,113,107,103,95,88,89,93,104,116,135,151
    ,156,159,151,147,154,153,148,140,129,121,116,117,118,122,129,135,139,135,126,117,110,106,100,92,89,91,99,109,127,145,154,161
    ,154,148,153,154,152,145,135,125,118,118,118,121,126,133,139,138,131,122,114,110,104,95,91,90,96,103,119,137,150,161,156,150
    ,152,154,154,149,141,130,121,119,118,120,124,131,138,140,135,127,118,114,108,100,93,91,94,98,112,129,144,159,159,152,152,154
    ,156,152,146,136,125,122,119,119,123,128,136,140,139,132,123,118,112,105,96,93,94,95,106,121,137,155,159,155,153,153,156,155
    ,150,141,130,124,120,120,122,125,134,140,142,137,127,122,116,110,100,95,95,93,102,114,128,149,158,157,154,153,156,156,154,146
    ,135,127,122,120,121,123,131,137,142,140,132,125,119,114,104,98,96,92,98,107,120,141,154,157,154,153,155,156,156,150,140,131
    ,124,121,120,121,127,134,141,142,136,129,122,118,108,100,97,91,95,102,112,132,148,156,154,153,154,155,157,153,144,135,127,123
    ,120,120,124,130,139,142,139,132,126,122,112,104,99,92,93,98,105,122,141,153,154,153,154,154,157,155,148,139,130,125,121,119
    ,121,126,136,141,142,135,128,125,116,107,102,94,93,95,99,113,132,147,153,153,153,153,156,156,151,143,133,128,122,119,119,122
    ,131,138,142,137,131,128,119,111,104,95,93,93,95,104,122,140,149,151,152,151,154,155,153,146,136,130,123,120,118,119,126,134
    ,141,138,132,131,122,114,107,98,94,92,92,97,112,131,144,149,151,150,152,153,154,148,139,132,125,121,118,117,122,129,139,139
    ,134,133,125,118,111,101,95,92,91,93,103,122,137,146,149,149,151,152,154,150,143,135,127,123,118,116,119,124,136,138,135,134
    ,128,121,114,104,97,93,91,90,96,113,130,142,147,148,149,150,153,151,146,138,130,125,119,116,116,120,132,137,136,136,130,124
    ,118,108,100,95,92,89,90,105,121,136,144,146,148,148,152,152,148,142,133,127,121,117,115,116,127,134,136,137,133,127,122,113
    ,104,97,94,89,88,97,113,130,140,145,147,147,151,152,150,145,137,131,123,120,116,113,123,131,135,137,135,131,125,118,108,100
    ,97,91,87,92,105,123,135,143,146,147,150,152,152,149,141,134,126,123,118,113,120,128,134,138,137,134,129,123,113,104,100,94
    ,89,89,99,116,130,140,145,147,150,152,154,151,145,139,130,126,121,114,117,125,132,137,138,137,133,128,119,109,104,98,92,88
    ,94,109,123,137,143,146,149,151,155,153,149,143,134,130,125,117,117,123,129,136,139,139,136,132,124,114,108,102,96,90,91,103
    ,117,132,141,146,149,150,155,155,153,148,138,134,129,121,117,121,127,134,139,140,139,136,130,119,112,106,100,93,90,97,110,126
    ,137,144,148,149,155,155,155,152,142,137,133,124,119,119,125,132,138,141,140,139,134,125,116,110,105,96,90,93,104,120,132,141
    ,147,148,153,155,156,155,146,140,136,128,121,118,122,128,136,140,140,141,137,130,120,114,109,100,93,91,98,112,126,137,144,147
    ,152,154,156,157,149,143,139,132,124,119,120,125,133,138,140,142,140,134,124,116,112,104,96,90,93,105,118,132,140,144,150,152
    ,154,157,151,145,141,135,127,120,119,122,129,135,138,141,141,137,128,120,115,107,100,91,90,99,111,126,136,141,147,149,152,157
    ,153,147,143,138,131,123,119,119,125,132,136,140,142,140,132,123,118,111,104,94,89,95,104,120,130,137,145,147,150,155,155,149
    ,144,141,134,126,120,118,122,128,133,138,141,142,136,127,121,114,108,97,90,91,98,112,125,133,142,144,148,153,155,151,146,143
    ,137,129,122,118,120,125,131,135,139,143,138,131,123,117,112,101,93,90,93,105,119,128,138,142,145,151,154,152,147,144,139,133
    ,125,118,118,121,127,132,137,142,140,134,126,120,115,105,96,90,89,99,112,122,133,139,142,148,153,153,148,146,141,135,128,120
    ,117,118,124,129,134,140,141,138,129,122,118,109,100,92,87,94,105,116,128,136,139,144,151,152,150,147,143,138,131,122,118,116
    ,121,125,130,137,140,140,132,125,121,113,105,95,87,90,99,110,121,132,136,141,148,151,150,148,145,141,135,126,119,116,119,123
    ,127,134,139,141,135,127,123,116,109,99,89,88,93,103,115,127,133,137,145,150,151,149,146,143,138,130,122,117,117,120,124,130
    ,137,142,138,131,127,120,114,105,93,89,91,99,110,123,131,135,142,148,151,151,147,145,141,134,125,118,117,118,122,127,134,141
    ,140,134,130,124,118,111,99,91,91,95,105,118,127,132,139,146,150,151,148,147,143,139,130,122,119,118,121,125,131,141,142,138
    ,133,127,122,115,104,93,90,91,99,111,123,128,135,142,148,151,150,148,145,142,134,125,120,118,120,123,128,139,143,141,137,131
    ,126,121,111,98,93,91,95,107,119,126,132,139,147,151,151,150,148,145,139,130,124,120,121,122,125,136,142,143,140,135,130,126
    ,117,105,97,93,93,103,115,123,130,136,145,150,152,151,150,148,143,135,127,121,121,121,123,132,140,144,143,137,132,129,122,111
    ,101,95,92,99,109,119,127,133,142,148,152,152,151,150,146,140,131,124,122,122,121,128,138,143,145,141,135,133,127,117,106,99
    ,92,96,105,115,123,129,138,145,151,152,151,151,148,144,135,127,124,122,120,125,134,141,145,143,138,135,131,122,112,103,94,94
    ,100,111,119,125,133,141,148,151,151,151,149,146,139,129,125,122,119,121,129,137,143,144,139,137,134,126,117,107,98,93,96,106
    ,115,122,129,138,145,150,151,152,151,149,143,133,128,125,121,120,126,134,142,146,142,139,137,131,123,113,103,95,94,102,110,118
    ,125,133,141,147,150,151,150,150,146,136,130,126,121,118,121,129,138,144,142,139,138,132,126,116,106,96,92,97,105,113,120,127
    ,136,143,148,149,149,150,148,139,132,128,122,119,119,124,134,142,143,141,140,136,130,122,112,101,94,95,102,110,117,124,133,140
    ,146,149,149,150,150,143,135,131,125,120,118,120,129,138,142,141,140,137,132,125,116,105,95,93,97,104,112,118,127,135,142,147
    ,147,148,150,145,137,132,126,121,117,117,123,134,139,140,140,138,134,128,120,110,98,93,94,101,107,114,122,130,138,144,146,147
    ,150,147,140,134,129,123,119,115,120,129,137,140,140,139,136,131,125,115,103,95,93,98,104,110,117,126,134,142,144,146,149,148
    ,143,137,132,125,121,115,116,125,133,138,140,140,138,133,128,120,108,98,93,95,100,106,112,121,129,138,142,143,148,148,144,139
    ,134,128,123,116,114,120,129,136,139,140,139,135,131,124,114,102,94,94,97,103,109,116,124,135,140,142,146,149,146,142,136,131
    ,126,119,114,117,125,133,138,140,140,137,134,129,120,108,98,95,96,101,106,113,120,131,138,141,145,149,148,145,139,134,129,123
    ,116,115,122,130,136,139,141,139,136,132,125,114,103,96,96,98,104,110,116,126,135,139,143,148,148,147,142,137,132,126,119,115
    ,119,127,133,138,141,140,138,135,130,121,108,100,97,97,102,107,113,122,132,137,141,147,149,149,145,140,136,130,123,116,117,124
    ,130,137,141,142,140,138,135,127,115,104,99,98,101,106,110,118,129,135,140,145,149,150,148,143,139,134,127,119,116,121,127,135
    ,140,142,141,140,137,131,121,109,102,98,100,104,107,114,125,132,137,143,147,150,149,145,142,137,131,123,117,120,125,132,138,142
    ,143,141,140,136,127,116,107,101,100,103,105,111,121,129,135,140,146,150,151,148,144,140,135,127,119,119,122,128,136,141,143,142
    ,141,139,132,121,111,103,100,102,103,107,116,124,131,137,143,148,150,149,145,142,138,130,122,118,120,125,133,139,143,143,142,142
    ,136,127,117,107,101,102,102,104,112,121,128,134,140,146,150,150,147,144,141,134,126,119,119,122,129,136,141,142,142,142,139,131
    ,122,111,103,102,102,102,108,116,124,131,136,143,147,150,147,144,143,137,129,121,118,119,125,132,139,142,141,143,141,135,127,116
    ,106,103,102,101,105,112,120,127,133,140,145,149,149,146,144,141,133,125,119,119,122,129,136,141,141,142,143,138,132,121,110,105
    ,102,100,102,108,116,123,129,136,142,147,148,146,144,142,136,128,120,118,119,124,132,138,140,141,142,140,135,126,113,107,103,100
    ,100,104,111,118,126,131,138,145,147,146,144,143,138,131,122,118,117,120,128,135,138,140,142,141,138,130,118,110,105,101,99,101
    ,108,114,122,127,134,141,146,146,144,144,141,134,126,119,117,117,124,132,136,139,141,141,140,134,123,114,107,103,99,99,104,110
    ,118,124,130,138,144,145,144,144,143,137,129,121,118,116,120,129,134,137,140,140,141,138,128,118,110,105,100,99,102,107,114,120
    ,126,134,141,144,144,144,144,140,133,125,119,116,117,125,131,135,139,140,142,140,133,123,114,108,102,99,100,104,110,117,122,130
    ,138,143,144,144,144,142,136,129,122,117,116,122,128,133,138,139,142,142,137,127,118,112,105,101,99,102,107,114,119,126,135,141
    ,144,144,145,144,139,132,125,119,115,119,125,131,136,137,141,142,141,132,123,116,108,103,100,101,105,111,117,122,131,139,143,144
    ,144,146,142,136,130,122,116,118,122,129,135,137,140,143,143,137,128,121,113,106,102,100,103,109,114,119,128,136,142,144,144,146
    ,145,139,134,126,118,118,120,127,132,136,138,144
};


#endif