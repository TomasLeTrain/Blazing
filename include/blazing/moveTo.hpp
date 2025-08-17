class moveTo {
	private:
	units::V2Position target;

	void execute(){
		// actually execute
	}
	
	public:
	moveTo(float x,float y) :
	target(to_in(x),to_in(y)) {}
	
	moveTo& reverse(){
		// do stuff
		return this;
	}
};
