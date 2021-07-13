#ifndef States_h
#define States_h

#include <falcon_prog/state_machine.h>

class StateMachine;

class State {
	public:
    State(StateMachine *machine);
  	virtual void stateUpdate();
  	virtual void onInput( uint8_t input);
  	virtual ~State();

	protected:
    void setState( State *state);
    StateMachine *machine;
};

class StartState : public State {
	public:
    StartState(StateMachine *machine);
  	void stateUpdate();
  	void onInput( uint8_t input);
};

class RunState : public State {
	public:
    RunState(StateMachine *machine);
  	void stateUpdate();
  	void onInput( uint8_t input);
};

#endif
