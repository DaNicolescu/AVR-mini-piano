#ifndef SAMPLE1_H
#define SAMPLE1_H

#include <avr/pgmspace.h>

int sample1_size = 7995;

const unsigned char sample1[7995] PROGMEM = {
    128,128,129,130,132,129,126,125,139,132,115,121,128,127,120,127,126,124,126,126,119,121,131,135,133,114,122,132,127,120,129,141
    ,131,121,123,121,111,120,132,141,132,123,114,98,104,106,95,99,125,148,183,173,132,126,127,120,125,178,170,144,162,145,138,138
    ,119,114,128,110,82,95,119,117,118,112,103,151,187,161,120,109,130,167,166,139,142,150,127,105,117,117,113,120,126,131,127,120
    ,118,139,156,133,118,120,132,139,109,82,65,76,81,56,77,113,131,131,125,119,109,107,90,92,123,132,121,118,121,130,127,106
    ,118,153,138,117,139,150,138,139,162,193,201,179,143,124,129,136,154,156,180,217,183,131,121,125,144,143,140,154,159,145,114,128
    ,115,76,104,140,132,105,72,52,69,106,105,100,127,133,119,107,87,98,144,145,120,141,163,145,144,130,113,112,91,97,156,156
    ,97,89,102,127,137,125,117,141,169,166,155,139,126,112,110,122,131,142,153,141,122,103,83,80,117,148,142,140,121,109,118,109
    ,106,124,137,133,119,84,56,67,81,113,165,167,153,172,168,134,98,98,138,173,162,144,153,140,112,108,98,115,150,148,143,153
    ,163,154,144,152,157,154,140,131,160,187,178,161,175,165,131,145,164,150,144,129,107,127,136,115,104,106,109,136,121,64,41,60
    ,72,87,110,115,129,118,84,74,79,85,132,168,151,132,139,156,164,141,123,135,150,163,153,115,87,88,109,129,131,127,136,152
    ,137,124,125,128,141,151,139,128,131,134,152,156,130,112,111,121,142,141,133,150,155,145,136,138,160,161,159,145,132,103,65,65
    ,68,84,119,129,126,134,135,136,119,105,123,145,161,159,136,115,103,105,113,123,116,115,133,123,108,131,138,144,172,156,129,123
    ,134,158,189,187,174,182,180,173,169,151,133,136,158,166,143,127,126,140,132,109,106,107,87,56,46,66,75,93,117,101,79,72
    ,66,79,118,121,106,110,120,147,163,127,110,143,172,155,128,113,100,104,103,102,119,117,116,133,132,132,123,131,159,168,154,124
    ,121,147,161,160,156,155,146,127,126,133,140,143,149,163,156,140,145,155,167,168,147,122,119,99,59,57,75,90,117,109,102,137
    ,135,99,91,115,126,135,131,109,107,112,100,105,112,110,107,97,93,99,120,139,143,148,142,121,125,150,173,174,170,173,206,216
    ,170,143,155,167,169,164,148,134,143,153,141,134,120,123,129,93,53,40,52,92,116,100,68,65,89,102,89,65,84,118,133,129
    ,128,127,132,143,156,158,147,136,126,128,119,111,110,122,139,132,120,118,133,151,152,149,156,145,129,132,140,156,165,161,143,137
    ,127,112,124,137,141,146,145,131,132,138,146,164,166,150,137,118,97,67,41,49,82,107,108,105,104,113,107,97,104,122,129,134
    ,129,114,113,111,113,124,110,92,87,83,103,138,146,125,117,134,149,137,128,140,167,189,203,200,188,176,174,176,178,175,162,159
    ,149,146,151,148,143,161,164,121,76,58,69,97,98,82,84,94,105,101,75,57,70,101,116,115,109,111,126,137,146,142,143,153
    ,159,137,117,109,112,130,132,121,114,115,121,137,146,151,148,142,137,136,131,137,154,162,161,142,122,113,123,125,130,143,137,134
    ,128,117,135,154,159,160,165,156,140,114,66,48,65,89,107,107,97,107,111,98,93,104,120,127,136,135,112,108,129,151,130,91
    ,81,92,121,124,116,119,127,144,150,129,118,128,153,174,180,188,183,188,197,192,185,169,178,198,177,129,117,145,181,198,156,118
    ,110,106,90,73,66,80,100,100,98,99,82,60,71,88,100,98,93,102,122,117,116,130,148,163,155,141,130,121,111,120,133,132
    ,118,106,107,123,130,139,142,144,144,131,128,133,152,160,160,164,149,132,122,122,139,142,125,125,129,126,129,127,141,158,179,176
    ,153,142,117,88,59,53,89,104,87,88,96,100,88,87,116,128,123,106,102,128,146,150,128,109,108,112,106,95,103,118,135,139
    ,145,136,119,126,147,160,164,164,181,207,208,180,170,184,206,220,183,138,132,152,168,186,174,147,143,141,125,98,65,60,96,113
    ,93,81,82,84,89,70,66,82,94,93,89,100,106,114,135,147,154,151,135,129,127,129,132,133,132,120,104,99,104,130,141,135
    ,139,130,127,125,133,155,162,169,163,151,144,137,147,147,131,139,146,138,127,119,131,141,169,185,185,169,149,143,115,76,63,89
    ,110,112,90,69,79,106,122,119,97,95,117,136,137,138,137,129,136,129,111,94,85,104,136,136,121,119,125,136,136,115,119,155
    ,185,187,171,164,175,203,208,203,189,164,154,151,151,151,156,167,166,160,137,106,94,87,84,92,97,90,88,92,95,84,78,77
    ,86,101,91,85,86,92,123,138,145,146,140,146,134,128,140,141,143,134,120,108,95,112,125,139,141,131,122,114,123,145,157,148
    ,153,164,159,143,137,148,144,139,146,137,133,116,104,117,141,160,170,177,171,168,146,97,78,100,106,103,86,68,76,99,103,100
    ,95,82,99,123,119,112,124,135,151,147,105,88,100,111,109,102,105,119,141,125,101,103,109,128,160,160,140,146,160,189,212,197
    ,175,180,193,171,142,132,141,179,189,166,152,138,126,113,92,94,91,96,95,86,98,86,77,79,93,92,87,82,73,77,90,108
    ,128,142,137,134,135,128,135,141,137,143,136,122,96,90,115,131,128,121,121,118,115,124,135,146,146,146,166,160,143,138,141,152
    ,162,149,121,126,131,112,102,110,143,189,192,159,151,142,124,122,109,91,84,83,79,95,96,77,84,99,102,96,90,86,121,154
    ,139,131,125,117,119,106,88,86,106,125,132,128,98,85,113,124,140,130,125,141,150,169,173,184,197,197,195,190,170,151,144,157
    ,180,192,178,154,152,154,127,104,98,103,115,101,92,88,88,92,87,91,96,88,79,72,70,84,111,121,122,139,132,124,132,140
    ,156,160,145,128,127,113,108,129,128,127,131,123,116,129,122,120,146,154,158,162,148,139,161,158,150,159,152,148,145,122,88,92
    ,121,151,187,174,142,152,161,155,134,97,88,103,115,92,66,80,89,109,99,72,77,90,112,122,125,131,133,146,146,114,90,87
    ,102,129,135,112,99,105,117,115,116,115,122,143,139,136,153,169,190,209,208,196,181,181,169,160,167,177,186,188,183,165,158,141
    ,117,121,121,114,110,99,84,88,96,92,99,95,83,82,78,60,75,105,114,126,124,107,119,144,140,153,163,134,125,129,116,121
    ,124,112,119,141,124,101,109,108,130,154,133,137,158,156,151,143,145,151,175,179,150,135,104,88,109,125,140,147,155,168,173,162
    ,141,128,135,126,108,100,86,90,101,105,93,79,83,89,97,105,92,113,149,160,154,125,106,107,120,116,108,117,118,111,113,114
    ,111,107,120,124,125,128,121,131,166,198,203,196,183,184,189,174,151,165,181,187,195,181,166,164,148,127,133,135,116,113,98,81
    ,101,94,86,109,108,82,73,71,61,77,103,101,117,111,89,120,138,143,148,144,142,138,138,117,107,129,126,127,127,111,109,113
    ,116,121,121,133,142,156,161,134,124,147,180,197,175,138,129,139,129,97,101,124,150,173,166,146,154,164,160,146,123,97,99,115
    ,106,101,88,86,95,104,84,60,77,107,126,146,148,132,126,127,120,109,113,110,118,124,111,107,107,100,113,127,113,104,118,114
    ,120,160,174,189,202,181,183,193,169,157,166,171,180,206,185,156,175,160,138,142,128,130,123,99,85,94,105,96,111,109,78,71
    ,72,66,87,98,91,86,95,103,108,123,131,142,158,147,122,120,122,127,131,128,118,122,128,119,109,104,96,123,157,152,133,129
    ,128,137,165,173,169,173,169,155,143,114,92,104,134,143,148,146,149,165,181,160,124,121,128,121,111,100,90,104,110,99,79,65
    ,65,85,102,108,132,138,122,133,135,112,112,116,119,126,116,98,101,110,108,120,121,92,105,114,96,107,141,164,179,191,187,183
    ,180,165,157,174,171,179,197,187,167,171,165,146,146,152,138,121,109,87,91,109,113,109,97,73,74,84,85,71,77,91,89,92
    ,87,92,117,138,143,146,140,121,124,143,132,116,120,130,140,139,108,84,99,129,138,139,125,122,139,146,134,136,166,184,186,181
    ,153,123,127,122,116,113,117,131,154,159,157,159,147,147,155,130,105,113,108,108,116,113,89,77,69,69,81,81,90,124,134,123
    ,130,127,113,117,133,119,109,115,102,99,116,114,120,113,91,96,113,96,87,124,155,167,186,187,169,178,174,166,166,176,183,192
    ,193,174,169,173,163,163,169,149,116,105,108,109,121,113,96,102,93,77,82,84,77,85,102,82,68,82,97,116,131,124,132,149
    ,149,133,124,122,123,148,152,133,117,111,107,114,123,123,117,131,137,138,123,109,140,173,186,183,169,151,157,155,129,98,108,124
    ,136,148,142,150,160,163,163,154,124,107,126,130,113,124,118,90,90,85,64,67,78,90,120,126,109,121,130,114,128,141,120,108
    ,116,110,104,113,122,116,114,103,101,104,89,87,109,143,159,171,185,176,171,178,167,164,176,189,193,191,173,165,186,191,170,163
    ,147,126,121,125,117,110,119,116,107,92,77,82,99,100,92,78,75,77,88,98,94,98,129,151,151,135,114,118,131,149,145,131
    ,124,131,132,112,102,115,122,135,141,123,107,115,129,140,157,161,174,184,168,159,158,126,109,126,125,116,131,139,146,165,164,156
    ,148,129,124,136,126,117,129,131,108,90,85,72,63,77,96,106,105,112,113,123,130,127,135,125,113,116,120,103,108,129,127,113
    ,112,101,96,95,81,97,128,143,157,182,182,163,162,171,174,185,190,182,175,178,187,195,188,180,165,159,149,123,123,125,126,133
    ,123,96,81,95,104,100,92,76,89,94,79,76,71,74,111,136,131,127,129,119,126,140,130,130,136,140,143,124,89,108,134,127
    ,128,125,109,112,127,118,120,137,158,182,182,168,160,160,144,126,130,116,105,121,148,149,147,161,158,146,144,132,124,131,128,137
    ,141,118,92,93,80,64,82,94,88,94,106,110,120,124,125,132,132,115,114,116,106,106,127,133,119,104,104,105,86,82,83,100
    ,137,157,164,159,153,157,175,183,169,174,178,173,184,186,185,194,190,184,165,136,125,146,142,131,131,110,103,109,104,88,86,91
    ,100,102,83,63,65,68,83,109,107,104,129,135,124,117,120,135,141,148,139,129,114,111,123,128,124,120,127,122,109,112,113,107
    ,129,160,170,167,168,170,169,161,140,128,118,107,126,141,139,142,160,164,150,139,134,133,127,139,148,144,125,103,98,93,82,74
    ,82,93,85,92,111,107,114,132,131,125,123,114,110,114,110,123,131,121,116,116,101,77,75,81,103,127,130,145,151,149,167,172
    ,168,170,180,178,168,171,179,197,206,201,177,155,150,158,150,135,128,131,135,121,110,94,84,97,115,109,89,78,74,72,76,75
    ,79,99,112,126,129,107,105,131,143,134,138,136,127,126,120,119,120,124,130,132,121,104,106,113,110,123,142,154,154,171,187,167
    ,161,155,138,122,115,116,130,141,141,154,169,152,128,136,138,131,142,150,144,135,117,106,100,87,78,82,88,82,91,95,101,121
    ,124,127,123,118,123,122,107,101,120,135,134,126,114,94,81,83,90,90,99,117,132,144,147,148,160,174,177,173,156,150,179,197
    ,191,200,194,179,176,170,156,147,137,143,153,143,116,101,104,99,113,113,97,96,94,87,68,60,74,87,98,103,114,112,104,114
    ,126,137,134,131,145,135,121,124,118,121,135,144,131,114,110,110,113,107,107,129,147,152,172,180,175,167,156,156,136,113,117,128
    ,139,143,155,156,143,141,134,136,144,141,145,152,140,128,120,106,101,94,89,84,74,80,96,101,105,120,125,123,127,128,110,101
    ,104,123,147,137,125,116,101,94,94,79,83,105,114,119,119,130,153,167,168,167,161,151,167,180,180,186,186,204,203,184,170,155
    ,154,158,165,151,127,130,124,113,112,101,111,117,102,102,89,65,67,85,87,85,98,104,103,109,115,118,130,130,142,146,126,121
    ,118,131,134,127,137,138,120,111,117,110,102,104,115,132,151,157,165,178,174,170,162,139,121,121,124,136,147,146,152,149,135,136
    ,143,133,138,155,152,144,133,128,125,115,97,91,84,75,87,90,84,98,114,125,125,124,115,106,106,112,128,126,136,141,124,110
    ,89,86,99,92,92,96,94,107,133,151,150,159,160,155,164,163,161,167,177,199,215,194,173,176,172,166,163,162,146,145,144,125
    ,117,106,112,124,113,114,108,83,75,78,82,82,80,86,104,108,91,95,120,129,130,135,133,130,120,123,130,129,129,132,132,134
    ,123,107,107,105,100,102,119,127,145,165,172,173,174,165,143,129,120,130,139,132,147,155,142,132,133,139,137,141,147,146,145,140
    ,142,130,114,112,95,89,88,79,76,72,93,118,119,111,113,122,110,102,106,115,134,143,136,128,106,96,110,98,91,86,76,87
    ,106,127,127,129,154,163,161,149,142,159,166,179,200,193,186,192,183,173,171,171,163,162,154,142,135,117,113,126,130,115,112,111
    ,102,83,73,78,83,90,87,90,93,93,90,109,128,124,129,128,128,130,121,122,129,129,132,142,139,120,118,116,102,98,101,106
    ,112,133,160,171,172,169,172,157,131,133,133,134,143,147,152,139,133,140,141,137,139,144,143,147,153,147,128,127,127,119,100,80
    ,85,78,74,94,105,112,119,118,115,105,101,110,121,132,139,137,123,118,125,115,97,90,90,86,87,99,102,112,138,153,153,146
    ,141,148,156,164,173,186,194,188,191,193,180,171,179,181,169,159,144,135,140,124,121,130,129,127,116,104,91,82,81,93,97,82
    ,85,91,85,95,104,114,123,120,133,135,121,120,130,128,120,137,147,133,131,129,119,113,98,96,102,98,123,152,158,165,175,173
    ,153,142,141,138,141,139,148,150,131,141,147,135,129,136,148,145,145,149,143,143,143,138,123,106,102,86,75,76,86,103,109,114
    ,120,102,92,110,117,116,124,127,126,142,138,119,110,105,104,102,86,72,85,104,103,126,148,136,138,139,140,157,156,156,177,195
    ,188,180,183,183,191,183,171,180,167,151,145,136,130,123,132,142,127,116,113,95,83,96,99,80,83,89,82,87,83,95,115,107
    ,113,134,126,114,124,131,118,124,139,131,139,138,131,132,110,103,104,88,87,111,139,144,155,174,168,150,144,152,145,134,138,145
    ,148,146,138,137,134,134,141,140,135,143,154,138,143,159,138,130,129,105,93,79,70,88,107,106,101,107,96,100,120,107,101,120
    ,126,132,138,132,123,123,118,110,109,81,68,84,90,108,113,120,138,137,129,129,139,156,158,162,180,185,174,177,195,183,176,190
    ,182,175,166,142,139,131,131,150,138,124,129,125,99,91,104,96,85,88,87,88,79,79,96,100,99,114,127,117,118,126,124,118
    ,118,135,135,128,144,148,130,117,113,103,85,82,101,128,139,141,161,164,149,158,152,137,144,150,143,145,151,140,136,137,134,142
    ,139,129,143,146,140,145,150,153,145,138,115,88,86,86,97,98,90,104,107,102,101,99,106,106,111,126,130,128,129,131,134,134
    ,114,96,98,87,67,89,101,99,123,128,121,122,126,139,150,148,153,175,180,169,182,185,175,187,200,193,172,168,159,139,138,147
    ,145,138,135,135,128,111,101,109,107,87,90,101,84,72,91,94,85,101,113,115,119,114,122,129,115,115,133,129,127,146,150,137
    ,133,123,97,93,93,95,113,122,138,157,155,151,155,156,144,144,153,147,147,149,143,140,142,139,134,138,140,137,139,131,142,169
    ,165,144,137,128,109,98,96,92,93,101,95,107,115,86,87,112,113,109,113,118,125,135,134,135,139,125,113,104,82,75,92,98
    ,95,112,123,112,114,130,133,131,140,155,167,170,165,170,179,175,190,205,192,180,180,169,144,144,157,148,138,144,143,134,118,110
    ,120,108,89,98,105,86,76,90,90,85,98,103,108,115,116,118,123,120,116,121,121,128,153,151,139,140,132,112,93,92,96,102
    ,114,125,144,154,145,149,153,145,153,154,142,146,156,150,135,137,145,145,143,131,125,135,138,142,161,169,148,138,142,129,109,93
    ,92,102,106,100,99,104,94,90,108,108,102,113,117,114,126,141,140,140,138,122,102,90,90,95,92,87,103,119,114,108,117,125
    ,122,134,157,154,151,166,167,163,174,190,198,195,184,185,179,151,148,162,154,138,144,153,136,129,124,117,116,104,100,101,91,83
    ,91,87,84,95,99,96,106,119,119,117,112,113,121,117,122,145,152,143,144,140,119,105,99,92,92,105,125,131,135,146,146,150
    ,148,139,155,159,143,146,152,142,141,154,147,137,130,121,133,144,139,145,162,156,149,151,133,110,108,108,101,101,103,101,99,96
    ,94,100,106,107,103,104,107,118,144,146,135,139,134,109,99,104,88,84,94,100,116,106,94,114,121,110,129,152,143,145,159,157
    ,156,170,182,192,198,187,188,184,160,158,163,149,143,155,150,137,135,131,126,119,110,109,106,91,88,98,90,79,86,95,98,103
    ,115,115,109,116,114,104,112,126,137,145,143,143,154,132,97,107,105,89,99,109,120,136,135,131,143,146,144,155,157,144,140,149
    ,149,151,156,143,133,133,134,130,126,134,146,156,162,152,148,143,125,112,115,111,101,111,103,89,97,100,99,104,102,93,98,102
    ,115,140,136,130,146,145,118,106,104,98,90,89,102,110,100,95,106,109,108,123,137,136,142,145,146,152,163,176,183,184,192,199
    ,182,167,169,166,156,148,155,159,137,135,145,137,123,116,112,104,106,98,90,92,82,85,95,86,95,120,112,98,109,112,108,105
    ,111,132,142,143,145,148,142,119,106,105,98,92,104,120,119,126,128,129,147,156,146,140,147,144,148,159,149,149,154,139,132,137
    ,125,120,136,141,148,158,157,152,145,128,121,128,121,108,108,99,91,102,107,103,94,96,99,95,95,107,122,134,133,143,151,126
    ,112,112,104,94,92,102,103,98,102,93,97,110,117,131,127,128,145,147,137,153,176,169,176,198,197,185,176,170,173,168,152,153
    ,157,144,147,155,137,126,127,126,110,105,108,103,93,83,84,91,96,93,103,118,105,102,113,102,98,113,123,129,145,148,148,151
    ,133,110,106,109,101,107,113,107,116,128,131,143,147,139,141,149,147,147,154,153,157,157,142,139,143,130,120,124,141,150,154,158
    ,152,144,138,137,142,128,107,109,109,104,104,109,102,96,102,95,92,93,100,117,126,135,145,149,134,122,126,108,94,107,109,98
    ,96,99,95,96,104,113,123,120,124,140,139,132,145,164,166,175,195,191,180,184,190,180,165,159,160,165,152,144,155,156,131,126
    ,134,122,118,114,104,96,91,88,87,93,99,102,110,107,104,105,105,100,102,116,124,136,152,153,148,135,118,121,120,110,101,100
    ,109,114,121,127,131,143,142,134,141,147,152,151,151,159,159,153,144,144,133,111,124,146,151,148,146,149,151,150,145,140,133,122
    ,115,112,109,109,111,107,97,107,100,80,90,102,105,109,127,148,147,137,127,126,120,106,108,113,98,95,104,96,91,101,108,107
    ,117,129,128,125,122,138,163,159,155,177,194,189,180,189,185,172,165,159,164,161,155,154,150,147,133,133,136,120,121,115,97,95
    ,92,86,92,96,100,104,106,104,104,105,90,89,107,119,135,144,144,142,141,134,125,122,112,106,106,99,104,121,123,122,131,136
    ,129,138,148,144,144,144,159,168,161,147,135,135,124,126,143,136,137,149,147,147,155,150,136,139,133,120,113,109,113,122,105,95
    ,109,103,82,83,97,93,100,126,136,141,137,127,132,126,117,114,107,101,98,107,98,77,92,108,113,109,111,121,119,122,131,140
    ,152,153,162,184,186,183,189,186,175,167,168,167,163,159,153,154,146,139,142,136,132,128,118,105,98,96,92,92,93,93,104,110
    ,109,104,94,83,89,108,110,119,136,141,141,136,138,137,127,121,107,105,102,102,118,120,113,119,132,134,135,137,129,133,157,162
    ,162,160,147,145,138,128,133,134,131,136,145,146,140,151,154,144,141,134,128,117,113,123,116,107,107,111,105,84,81,88,91,94
    ,110,132,130,123,138,140,126,118,114,118,115,103,97,92,92,92,101,106,101,108,113,111,115,124,134,136,138,156,173,180,181,187
    ,187,178,172,172,174,167,165,158,154,153,144,147,146,137,134,123,113,109,107,94,79,91,103,105,108,105,104,98,84,83,99,110
    ,109,123,137,133,138,145,141,138,120,105,112,114,110,105,106,113,124,131,124,126,131,125,134,151,165,159,151,160,152,141,137,131
    ,136,136,132,138,140,145,152,155,150,138,145,140,117,119,127,123,114,115,123,109,86,79,90,99,88,91,117,132,134,131,131,128
    ,129,130,124,116,108,105,97,91,97,103,102,97,104,114,109,111,117,124,130,130,145,167,175,175,184,186,176,178,181,174,177,174
    ,159,154,154,159,157,147,138,142,147,127,112,112,98,88,93,100,111,110,106,106,99,88,81,99,109,105,110,117,137,150,146,141
    ,131,131,129,116,116,111,102,106,116,129,126,116,121,127,128,128,142,158,160,159,158,157,148,141,146,140,128,134,141,140,141,152
    ,155,148,145,148,148,130,113,125,136,128,118,119,114,100,97,93,86,82,93,114,121,125,130,128,130,134,136,131,120,111,108,108
    ,97,95,105,96,94,107,108,107,109,112,118,116,121,139,156,163,170,184,178,171,182,189,185,173,166,171,166,161,155,154,153,145
    ,150,148,138,127,113,106,94,88,104,112,113,110,100,96,94,98,96,90,93,101,118,132,140,141,133,138,141,133,124,110,110,113
    ,105,115,127,117,112,123,123,115,121,137,152,156,150,156,163,153,144,149,145,126,130,141,139,138,139,149,153,152,152,139,125,129
    ,138,133,121,124,129,124,110,95,94,87,77,91,107,109,117,120,123,129,135,136,131,125,117,119,110,98,106,102,92,92,103,112
    ,103,101,113,109,102,106,130,152,151,155,163,176,180,175,182,182,175,175,174,170,162,158,154,153,153,151,151,148,140,126,99,89
    ,103,111,111,105,100,106,108,101,96,91,81,88,105,106,117,130,130,135,141,145,134,120,119,119,113,103,110,129,121,107,111,118
    ,117,111,127,145,145,142,153,161,157,153,146,137,141,140,131,130,128,145,157,144,144,151,145,132,132,136,129,126,126,134,137,113
    ,100,98,87,89,90,94,98,105,118,118,121,133,139,132,122,126,125,113,105,104,109,96,84,98,115,111,99,96,99,102,109,118
    ,132,140,142,158,168,170,174,174,175,182,182,171,168,172,165,154,146,147,167,167,150,142,128,111,106,112,111,102,103,105,112,113
    ,103,97,88,87,92,96,95,100,121,134,128,135,146,142,127,118,124,114,108,117,122,124,112,104,114,115,115,121,127,131,145,159
    ,150,149,156,153,152,142,134,134,127,133,146,149,141,140,153,150,139,133,133,128,125,138,142,132,121,111,104,98,91,92,93,90
    ,97,112,118,115,125,135,136,127,122,128,128,117,107,100,92,93,106,112,103,101,98,94,102,104,107,116,127,142,150,154,159,173
    ,179,169,173,181,182,182,174,160,148,150,164,171,165,151,144,141,127,117,115,108,103,104,117,116,105,107,107,97,88,89,91,93
    ,99,108,119,129,133,145,141,129,129,120,114,117,124,125,115,109,115,119,111,105,114,125,130,136,144,147,147,161,164,151,143,140
    ,133,130,139,145,139,136,144,159,155,136,132,137,132,130,138,146,140,125,119,116,109,97,90,94,93,91,101,108,117,128,127,119
    ,125,139,136,130,121,111,106,98,101,110,107,101,105,107,97,91,98,104,112,119,123,136,149,163,165,159,160,174,189,191,179,170
    ,164,155,156,170,175,158,152,158,154,140,121,115,119,112,106,112,115,115,113,107,99,100,94,86,90,97,102,106,118,134,147,143
    ,123,125,137,125,115,121,126,120,118,118,112,109,108,114,119,122,126,134,140,149,165,164,147,142,150,146,132,133,139,139,139,144
    ,156,158,139,128,139,140,131,137,141,142,143,127,114,116,115,104,90,82,91,106,106,109,120,116,113,129,145,142,128,120,122,115
    ,105,102,103,109,114,108,97,97,99,101,102,96,101,123,134,141,150,150,149,159,177,188,188,172,166,174,167,158,164,168,168,163
    ,157,154,151,138,121,120,118,113,112,114,116,119,115,100,100,105,96,83,84,95,103,112,126,135,135,129,132,135,126,116,121,132
    ,124,120,121,110,105,117,118,111,110,112,129,142,143,151,159,149,147,159,150,134,132,134,137,146,146,149,150,145,142,135,129,135
    ,144,141,139,138,136,129,124,123,110,85,83,102,104,100,103,102,110,121,124,133,139,134,132,127,111,104,114,111,107,108,102,105
    ,108,101,95,91,87,96,120,130,127,129,133,148,164,167,172,182,180,171,171,168,162,166,170,166,163,168,159,151,148,137,124,116
    ,116,117,119,116,111,115,115,107,108,97,77,85,97,93,101,119,125,130,131,132,133,124,115,129,139,121,114,118,116,116,118,111
    ,104,103,110,121,130,137,142,143,150,161,158,143,132,136,141,133,131,148,156,143,144,139,130,138,142,136,133,135,137,144,144,130
    ,122,110,93,101,105,92,91,100,105,103,105,111,132,144,133,124,124,119,118,117,107,105,106,104,113,118,101,81,84,97,100,106
    ,113,114,121,129,134,147,164,170,169,172,175,173,168,160,164,176,168,158,168,169,158,154,140,128,129,127,118,115,115,115,121,117
    ,113,114,99,81,89,98,88,86,105,128,129,124,123,126,129,129,131,126,124,121,117,124,125,119,109,95,106,123,110,108,129,143
    ,139,145,157,154,156,143,131,137,135,138,150,151,143,142,140,138,147,145,124,123,138,149,151,141,129,133,126,106,100,103,102,101
    ,99,93,95,106,113,123,139,134,120,126,137,128,113,105,105,115,120,114,109,105,92,88,95,96,106,110,103,110,125,129,133,154
    ,167,165,169,171,171,174,167,165,170,168,167,172,173,162,156,152,140,136,137,123,113,115,127,126,117,119,115,110,101,90,89,86
    ,88,108,120,118,117,124,123,131,146,126,114,127,131,130,126,120,120,119,109,106,112,107,108,126,130,128,144,154,158,159,146,131
    ,136,145,148,148,140,137,148,155,145,141,138,129,130,140,141,145,153,145,133,127,119,111,111,114,104,93,89,93,111,114,110,121
    ,130,133,136,135,124,116,117,112,113,125,121,111,104,98,96,96,97,100,104,102,101,112,118,127,147,157,157,158,169,177,168,167
    ,168,164,171,173,175,172,163,160,161,157,142,134,127,118,129,129,112,119,131,123,114,107,93,84,89,96,106,108,100,109,126,128
    ,129,132,122,122,134,130,123,128,128,123,119,112,109,110,108,105,113,116,120,141,158,151,146,141,139,147,149,140,130,143,151,146
    ,146,146,145,140,128,127,134,140,146,153,152,131,124,129,127,125,113,96,93,100,102,96,99,106,113,127,130,134,136,121,115,116
    ,119,121,120,123,113,105,107,95,93,101,103,98,92,97,102,111,118,129,148,146,147,167,173,164,161,161,165,178,176,160,166,176
    ,169,162,154,147,143,134,125,129,126,110,120,140,133,115,101,92,99,103,94,86,95,106,108,116,118,123,132,123,120,128,129,124
    ,129,135,124,113,117,116,117,109,93,94,112,132,133,132,138,145,151,143,139,148,140,131,139,147,147,142,149,149,140,129,118,130
    ,147,148,145,137,136,138,136,131,121,119,110,97,101,103,96,90,96,109,119,132,132,127,125,118,119,121,120,125,125,114,106,109
    ,102,97,106,103,94,91,91,98,107,109,112,129,146,150,156,155,157,168,165,162,170,170,164,168,180,175,160,157,153,157,151,125
    ,118,123,128,134,134,127,115,114,110,101,103,96,87,94,102,106,104,114,125,124,125,117,119,132,136,132,123,118,127,133,126,108
    ,102,103,98,110,122,123,124,131,146,148,144,143,144,147,137,130,140,151,155,154,145,132,130,135,134,137,143,142,141,142,143,141
    ,135,128,125,119,109,108,106,91,88,95,104,114,125,130,126,123,119,118,125,126,126,128,117,109,109,110,111,107,99,89,97,102
    ,91,92,98,106,127,137,138,143,149,159,166,168,158,157,170,174,177,175,165,163,172,177,161,139,132,128,133,137,131,130,130,128
    ,121,114,110,104,101,97,92,96,96,105,121,121,111,112,127,133,128,127,124,125,132,133,132,127,114,108,107,103,103,113,118,119
    ,128,136,138,144,154,151,136,129,132,148,156,154,151,143,138,138,139,135,135,141,139,138,148,148,141,135,132,136,132,120,113,108
    ,97,88,94,101,108,123,125,124,126,115,115,131,137,129,120,117,119,122,116,104,108,109,100,100,97,90,85,93,108,116,123,124
    ,130,154,164,155,147,154,170,174,171,165,166,175,176,177,177,163,147,141,138,140,135,132,130,138,138,118,112,124,122,105,89,87
    ,99,107,104,104,109,110,116,127,129,120,119,127,130,135,137,130,125,126,120,106,98,106,117,116,106,111,131,139,146,151,142,134
    ,132,135,145,153,154,150,143,140,143,146,134,131,138,136,142,147,145,141,135,135,140,142,131,117,114,104,92,89,92,110,122,119
    ,112,112,124,124,124,130,126,122,124,126,121,112,108,111,117,115,95,84,91,98,99,92,93,108,125,136,137,142,145,147,161,168
    ,164,159,161,166,177,185,175,170,172,164,148,136,136,144,145,133,126,133,133,126,129,121,102,94,100,104,100,99,97,103,113,117
    ,122,118,114,125,127,122,131,139,137,128,125,119,112,110,110,113,104,97,112,125,133,141,143,140,130,132,137,143,151,148,147,147
    ,144,145,143,136,131,133,141,140,143,142,137,140,139,135,141,146,135,115,101,94,94,103,107,107,108,109,116,121,122,119,119,128
    ,133,129,122,111,112,124,126,114,104,98,96,105,99,81,81,93,106,117,123,121,126,141,150,158,158,149,154,166,166,166,176,183
    ,180,175,164,145,146,152,150,139,125,131,142,138,132,129,119,108,109,110,100,94,95,101,105,103,111,116,117,118,116,115,120,135
    ,145,131,123,126,128,123,115,113,109,102,93,102,123,131,130,135,133,132,134,134,139,146,145,145,151,153,143,137,138,140,138,132
    ,136,145,145,139,129,130,144,155,151,137,120,105,105,105,102,104,97,100,118,120,110,107,114,128,137,130,113,116,127,127,123,116
    ,112,114,111,106,103,93,80,81,99,104,101,103,114,128,134,138,146,152,154,153,150,152,168,185,181,173,172,167,160,155,157,149
    ,133,131,140,146,139,129,131,129,122,116,109,101,101,102,96,96,105,113,116,111,106,112,117,119,131,140,133,123,124,130,134,127
    ,113,108,103,96,102,116,126,127,121,128,139,136,128,131,140,147,153,146,144,152,144,138,137,132,140,147,142,136,134,128,132,152
    ,158,147,137,127,125,121,106,96,100,110,112,108,102,103,115,125,126,126,120,118,125,133,130,115,111,123,130,121,104,99,98,94
    ,92,91,93,96,105,110,111,122,136,145,149,146,142,142,153,171,182,173,165,176,180,170,163,155,148,146,142,141,142,140,138,140
    ,132,123,124,121,111,103,98,98,105,106,111,113,108,105,108,115,122,130,131,130,127,122,135,146,132,119,109,103,106,107,108,116
    ,122,125,130,131,125,131,137,134,138,147,152,155,152,144,135,133,142,154,152,135,127,130,138,145,144,142,148,153,149,132,114,107
    ,112,118,112,105,99,99,112,122,121,113,116,126,128,131,128,119,117,126,132,129,119,111,112,108,95,93,96,94,96,97,95,103
    ,122,134,138,139,134,137,146,153,164,168,163,171,183,181,172,165,163,160,150,140,144,152,143,138,140,138,137,131,124,121,108,96
    ,102,110,110,111,109,103,104,107,114,123,125,125,123,122,131,144,142,130,125,124,114,102,99,112,119,116,117,123,126,130,132,127
    ,126,131,145,159,160,146,132,134,144,156,153,136,131,137,142,134,129,136,146,156,158,146,129,122,124,125,122,108,96,101,109,116
    ,113,102,109,127,130,120,119,125,126,124,122,125,133,128,118,113,106,99,102,102,97,91,84,85,103,121,123,123,123,131,143,140
    ,136,151,165,164,164,170,180,183,169,160,164,157,145,146,149,146,139,138,141,142,138,130,122,109,103,105,110,115,114,103,94,98
    ,114,122,111,107,120,127,122,122,135,142,138,132,122,115,106,104,111,114,110,109,120,130,129,122,111,114,138,155,150,138,140,142
    ,144,145,142,145,146,143,140,133,122,125,140,150,153,147,137,136,141,137,123,112,110,111,109,104,102,105,108,110,120,121,116,119
    ,123,122,120,119,125,137,132,118,111,107,109,115,105,89,83,84,92,97,102,111,119,119,120,130,135,134,140,151,158,158,165,178
    ,179,170,167,163,159,154,150,148,143,138,140,144,145,143,141,122,102,108,119,116,110,105,101,99,104,107,111,112,106,114,119,112
    ,118,136,142,137,133,125,118,120,115,103,99,105,120,132,122,108,110,117,125,135,137,134,140,150,147,138,134,139,151,159,150,132
    ,122,123,136,148,142,133,142,149,147,145,133,123,127,123,115,110,102,103,110,106,106,114,120,118,119,119,115,116,124,135,137,123
    ,115,122,123,117,111,108,101,87,85,89,93,101,107,110,112,118,126,129,130,133,142,152,151,162,177,173,169,173,172,159,160,163
    ,151,141,133,141,159,156,141,136,128,120,121,122,114,112,114,107,99,102,108,114,113,109,109,107,109,125,139,135,125,131,144,141
    ,123,105,98,107,122,126,118,112,110,116,122,119,121,129,135,146,154,137,126,138,151,163,156,138,134,137,134,136,137,135,138,147
    ,148,147,147,139,136,134,129,123,113,109,114,110,100,100,119,132,119,105,111,124,126,124,131,131,127,126,123,128,127,119,114,105
    ,96,90,93,90,94,109,105,101,117,126,124,122,126,140,146,143,154,175,176,163,165,175,178,173,161,147,141,146,154,155,152,149
    ,142,135,130,128,126,120,117,121,112,93,99,121,123,110,98,100,114,124,118,117,125,130,147,152,133,121,114,108,116,122,119,112
    ,115,121,121,111,102,119,140,141,136,131,131,140,144,151,152,148,149,146,139,135,133,134,136,142,144,142,143,146,153,143,125,131
    ,137,128,113,106,106,106,111,115,121,118,104,110,122,120,125,129,124,124,129,129,128,130,127,119,109,99,99,99,88,89,103,106
    ,98,101,119,125,118,117,122,134,148,154,155,157,160,171,184,177,168,165,151,143,151,157,150,151,155,147,138,127,128,142,133,115
    ,112,107,106,115,118,107,101,107,113,115,108,101,111,123,136,148,139,128,127,128,122,108,107,118,126,126,115,104,99,111,127,131
    ,129,123,127,136,136,142,145,143,147,156,153,135,130,138,138,138,132,133,145,147,144,144,139,135,142,143,127,113,111,109,108,111
    ,115,117,110,104,110,117,118,120,127,124,117,123,133,133,127,128,124,116,109,98,97,97,92,94,93,93,107,121,112,99,111,124
    ,130,139,139,142,148,155,173,181,172,166,169,162,146,145,153,158,163,153,138,131,139,153,144,121,113,118,123,117,109,107,103,109
    ,117,120,104,89,100,115,124,128,127,133,142,141,128,110,106,116,130,128,115,112,106,101,114,125,122,118,122,129,131,129,131,144
    ,150,146,146,148,145,141,138,132,129,137,142,139,140,140,142,142,141,146,147,132,113,116,121,110,109,115,115,111,103,106,118,120
    ,115,117,125,120,121,128,127,137,139,124,111,113,120,108,92,86,92,101,99,101,111,102,95,110,125,126,121,128,136,141,152,159
    ,168,177,180,169,149,146,157,166,161,147,147,147,142,149,153,138,121,124,134,131,115,99,104,120,121,115,108,99,102,110,106,101
    ,118,133,136,141,138,123,115,116,125,133,123,109,114,119,107,105,115,120,126,123,116,118,129,137,139,141,140,145,156,148,141,141
    ,131,131,140,146,138,131,136,141,148,145,143,151,140,120,119,124,121,116,114,111,111,114,108,107,116,120,122,117,109,120,135,134
    ,129,134,130,120,125,128,112,92,87,101,107,100,97,96,101,105,108,111,113,122,131,127,120,136,164,174,173,170,160,154,161,169
    ,165,153,142,151,163,154,141,142,140,141,145,135,117,113,118,117,118,114,112,117,113,106,104,93,94,118,138,139,129,127,128,128
    ,127,122,125,126,121,120,113,106,108,118,121,118,119,117,117,125,130,135,136,137,147,157,151,140,139,140,137,141,145,136,133,134
    ,138,145,151,151,146,139,131,129,130,120,117,125,119,107,105,114,118,114,117,116,108,115,127,130,127,126,130,137,137,130,126,113
    ,99,106,110,96,95,104,102,97,96,100,113,123,118,114,113,113,136,165,166,157,161,162,165,176,166,150,151,160,168,162,141,138
    ,152,158,148,141,135,124,126,127,119,111,111,121,129,123,103,89,95,103,117,128,125,125,134,132,125,125,130
};

#endif