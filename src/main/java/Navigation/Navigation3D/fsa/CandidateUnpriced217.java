package Navigation.Navigation3D.fsa;

/** No response value: this candidate may be deferred, not scored as dangerous. */
public final class CandidateUnpriced217 extends RuntimeException {
    public final String requestKey,inputSha,sourceStatus;
    public CandidateUnpriced217(String reason,String requestKey,String inputSha,String sourceStatus){
        super(reason);this.requestKey=requestKey;this.inputSha=inputSha;this.sourceStatus=sourceStatus;
    }
}
