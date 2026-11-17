package Navigation.Navigation3D.fsa;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import com.fasterxml.jackson.databind.ObjectMapper;

/** Shared on-demand evaluator. Cache misses solve the original response map. */
public final class ComponentProcessProvider211 implements ResearchResponsePort.Provider, AutoCloseable {
    public static ComponentProcessProvider211 CURRENT;
    private boolean readOnly220;private Thread owner220;public long screenRequests220,screenMissing220;
    public boolean isScreening220(){return readOnly220;}
    public static final class ScreenScope220 implements AutoCloseable {
        private final ComponentProcessProvider211 provider;private boolean closed;
        ScreenScope220(ComponentProcessProvider211 provider){
            this.provider=provider;
            if(provider.readOnly220||!FullRouteAudit211.phase.equals("SEARCH_EXECUTE"))throw new IllegalStateException("BOUND220_NESTED_OR_WRONG_PHASE");
            provider.readOnly220=true;provider.owner220=Thread.currentThread();
        }
        @Override public void close(){
            if(closed||!provider.readOnly220||provider.owner220!=Thread.currentThread())throw new IllegalStateException("BOUND220_SCOPE_OWNER");
            provider.readOnly220=false;provider.owner220=null;closed=true;
        }
    }
    public static ScreenScope220 readOnlyScreen220(){if(CURRENT==null)throw new IllegalStateException("BOUND220_PROVIDER_REQUIRED");return new ScreenScope220(CURRENT);}

    public long candidateDeferrals;public long controlStops;private String controlReason;private boolean finalAudit;
    public static void auditPhase(String phase){if(CURRENT!=null)CURRENT.finalAudit=phase.equals("FINAL_AUDIT")||phase.equals("DONE");}
    private final Process process;
    private final BufferedReader input;
    private final BufferedWriter output;
    private final Map<String,ResearchResponsePort.Response> memo = new LinkedHashMap<>();
    public final String revision;
    public long transportErrors;private boolean transportBroken;
    public long requests, memoryHits, diskHits, computed, failed, approximateResponses, physicalAttempts;
    public double invocationSeconds, backendWallSeconds, backendPythonCpuSeconds;

    public ComponentProcessProvider211(String python,Path script,Path cache,Path stderr)throws Exception {
        Files.createDirectories(stderr.toAbsolutePath().getParent());
        ProcessBuilder builder=new ProcessBuilder(python,"-u","-B","-X","utf8",script.toString(),
            "--cache",cache.toString(),"--jsonl-server","--tsv-rpc");
        builder.redirectError(stderr.toFile());
        process=builder.start();
        input=new BufferedReader(new InputStreamReader(process.getInputStream(),StandardCharsets.UTF_8));
        output=new BufferedWriter(new OutputStreamWriter(process.getOutputStream(),StandardCharsets.UTF_8));
        try{
            String[] ready=read(90).split("\t",-1);
            if(ready.length!=3||!ready[1].equals("READY")||!ready[2].matches("[0-9a-f]{64}"))throw new IOException("COMPONENT_HANDSHAKE");
            revision=ready[2];CURRENT=this;
        }catch(Exception error){stopBackend();throw error;}
    }
    private void stopBackend(){
        process.descendants().forEach(p->{try{p.destroyForcibly();}catch(Exception ignored){}});
        process.destroyForcibly();
    }
    private String read(int seconds)throws Exception {
        long end=System.nanoTime()+seconds*1000000000L;
        while(System.nanoTime()<end){
            if(input.ready()){
                String line=input.readLine();
                if(line==null)throw new EOFException("COMPONENT_EOF");
                if(line.startsWith("R1C\t"))return line;
            }else if(!process.isAlive())throw new EOFException("COMPONENT_PROCESS_EXIT_"+process.exitValue());
            Thread.sleep(5);
        }
        throw new IOException("COMPONENT_RPC_TIMEOUT");
    }
    @Override public synchronized ResearchResponsePort.Response response(ResearchResponsePort.Request r){
        if(controlReason!=null&&!finalAudit)throw new TruthBudgetStop211(controlReason);
        requests++;if(readOnly220){if(owner220!=Thread.currentThread())throw new IllegalStateException("BOUND220_WRONG_THREAD");screenRequests220++;}String key=r.key();
        if(transportBroken)throw new IllegalStateException("COMPONENT_TRANSPORT_DISABLED211");
        ResearchResponsePort.Response old=memo.get(key);
        if(old!=null&&(!readOnly220||ResearchResponsePort.isPriced(old.status))){memoryHits++;return old;}
        RequestContext211.publish(r);long started=System.nanoTime();
        ResearchResponsePort.Response result;
        try{
            double mu=Math.abs((r.waveFrom-r.heading+540.)%360.-180.);
            String query="{\"U_m_s\":"+r.U+",\"mu_deg\":"+mu+",\"Hm0_m\":"+r.H+",\"Tp_s\":"+r.Tp+
                ",\"configuration\":\""+r.configuration+"\"}";
            if(!r.configuration.equals(ResearchResponsePort.CONFIG))return ResearchResponsePort.Response.unavailable(ResearchResponsePort.Status.UNSUPPORTED_INPUT,"CONFIGURATION_UNSUPPORTED");
            query=query.substring(0,query.length()-1)+",\"rpc_request_id\":"+requests+"}";
            if(readOnly220)query=query.substring(0,query.length()-1)+",\"read_only_screen220\":true}";
            output.write(query);output.newLine();output.flush();
            String[] row=read(Integer.getInteger("nav.r1.rpcTimeoutSeconds",330)).split("\t",-1);
            if(row.length!=15||!revision.equals(row[13]))throw new IOException("COMPONENT_RESPONSE_CONTRACT");
            ResearchResponsePort.Status status=ResearchResponsePort.Status.valueOf(row[1]);
            String reason=new String(Base64.getDecoder().decode(row[2]),StandardCharsets.UTF_8);
            double[] sd=null;
            if(ResearchResponsePort.isPriced(status)){
                String[] cells=row[3].split(",",-1);if(cells.length!=4)throw new IOException("FOUR_SD_REQUIRED");
                sd=new double[4];for(int i=0;i<4;i++)sd[i]=Double.parseDouble(cells[i]);
            }
            String provenance=new String(Base64.getDecoder().decode(row[14]),StandardCharsets.UTF_8);
            Map details=new ObjectMapper().readValue(provenance,Map.class);
            if(!(details.get("rpc_request_id") instanceof Number)||((Number)details.get("rpc_request_id")).longValue()!=requests)throw new IOException("COMPONENT_REQUEST_ID_MISMATCH");
            if(readOnly220&&(!Boolean.TRUE.equals(details.get("read_only_screen220"))||Boolean.TRUE.equals(details.get("physical_solve_attempted"))||details.get("candidate_deferred_217")!=null))throw new IOException("BOUND220_READ_ONLY_ACK_OR_PHYSICS_CONTRACT");
            if(details.get("control_stop_211")!=null){if(finalAudit)throw new IOException("UNPRICED_FINAL_AUDIT211");controlStops++;controlReason=details.get("control_stop_211").toString();throw new TruthBudgetStop211(controlReason);}
            if(status==ResearchResponsePort.Status.CONTRACT_FAILURE)throw new IOException("BACKEND_CONTRACT_FAILURE211:"+reason);
            if(details.get("candidate_deferred_217")!=null){
                if(finalAudit||!"SEARCH_EXECUTE".equals(FullRouteAudit211.phase)||ResearchResponsePort.isPriced(status))throw new IOException("INVALID_CANDIDATE_DEFERRED217");
                candidateDeferrals++;invocationSeconds+=(System.nanoTime()-started)*1e-9;
                throw new CandidateUnpriced217(details.get("candidate_deferred_217").toString(),key,row[4],status.name());
            }
            result=new ResearchResponsePort.Response(status,reason,sd,row[4],row[5],row[6],row[7],row[8],provenance);
            if(status==ResearchResponsePort.Status.APPROXIMATED)approximateResponses++;
            if(Boolean.TRUE.equals(details.get("physical_solve_attempted")))physicalAttempts++;
            if(ResearchResponsePort.isPriced(status)&&Math.abs(result.cost-Double.parseDouble(row[9]))>1e-11)throw new IOException("JAVA_PYTHON_COST_MISMATCH");
            if(row[10].equals("1"))diskHits++;else{computed++;backendWallSeconds+=Double.parseDouble(row[11]);backendPythonCpuSeconds+=Double.parseDouble(row[12]);}
            if(!ResearchResponsePort.isPriced(status))failed++;
        }catch(CandidateUnpriced217 unavailable){throw unavailable;
        }catch(TruthBudgetStop211 stopped){throw stopped;
        }catch(Exception error){
            failed++;transportErrors++;transportBroken=true;stopBackend();
            throw new IllegalStateException("COMPONENT_TRANSPORT_ERROR211",error);
        }
        invocationSeconds+=(System.nanoTime()-started)*1e-9;
        if(readOnly220&&!ResearchResponsePort.isPriced(result.status)){screenMissing220++;return result;}
        memo.put(key,result);return result;
    }
    private String legacyStatistics220(){return "{\"candidate_deferrals\":"+candidateDeferrals+",\"revision\":\""+revision+"\",\"requests\":"+requests+",\"memory_hits\":"+memoryHits+
        ",\"disk_hits\":"+diskHits+",\"computed\":"+computed+",\"failed\":"+failed+",\"invocation_seconds\":"+invocationSeconds+
        ",\"backend_wall_seconds\":"+backendWallSeconds+",\"backend_python_cpu_seconds\":"+backendPythonCpuSeconds+",\"approximate_responses\":"+approximateResponses+",\"physical_solve_attempts\":"+physicalAttempts+",\"transport_errors\":"+transportErrors+",\"transport_broken\":"+transportBroken+"}";}
    public String statistics(){
        try{Map x=new ObjectMapper().readValue(legacyStatistics220(),Map.class);x.put("screen_requests220",screenRequests220);x.put("screen_missing220",screenMissing220);x.put("read_only_active220",readOnly220);return new ObjectMapper().writeValueAsString(x);}
        catch(IOException e){throw new IllegalStateException("BOUND220_STATISTICS_CONTRACT",e);}
    }
    @Override public void close()throws Exception {
        if(transportBroken){stopBackend();process.waitFor(15,java.util.concurrent.TimeUnit.SECONDS);input.close();output.close();return;}
        if(process.isAlive()){
            try{output.write("{\"command\":\"close\"}");output.newLine();output.flush();}
            finally{if(!process.waitFor(15,java.util.concurrent.TimeUnit.SECONDS))process.destroyForcibly();}
        }
        input.close();output.close();
    }
}
