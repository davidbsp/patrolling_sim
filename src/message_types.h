// Message types
#define INITIALIZE_MSG_TYPE 10
#define TARGET_REACHED_MSG_TYPE 11
#define INTERFERENCE_MSG_TYPE 12

// Message types for DTAGreedy algorithm
#define DTAGREEDY_MSG_TYPE 40 //IDleness message: msg format: [ID_ROBOT,msg_type,global_idleness[1..dimension],next_vertex]

// Message types for DTASSI algorithm
#define DTASSI_TR 41 //Task request, msg format: [ID_ROBOT,msg_type,next_vertex_index,bid_value]
#define DTASSI_BID 42 //Bid Message, msg format: [ID_ROBOT,msg_type,next_vertex_index,bid_value]

