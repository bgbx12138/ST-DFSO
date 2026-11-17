package Navigation.Navigation3D.safety;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/** Frozen dynamic-task preference; no physical validation or overall safe verdict. */
public final class ResolvedDynamicResponseCost {
    private ResolvedDynamicResponseCost() {}
    public static final String POLICY_ID = "R1_CONFIGURABLE_LITERATURE_TASK_WEIGHTS_V1";
    public enum Statistic {
        ROLL_CENTERED_SD_DEG("deg"), HEADING_CENTERED_SD_DEG("deg"),
        SWAY_CENTERED_SD_M_S("m/s"), SURGE_CENTERED_SD_M_S("m/s");
        public final String unit;
        Statistic(String unit) { this.unit = unit; }
    }
    public enum Mode { DIAGNOSTIC, EVIDENCE_GATED }
    public enum EvidenceStatus { NOT_ADMITTED, ADMITTED_WITHIN_SCOPE }
    public enum DomainStatus { UNKNOWN, WITHIN_DECLARED_SCOPE, OUTSIDE_SCOPE }
    private static final Statistic[] EXPECTED = Statistic.values();
    private static final double[] SCALES = {4.0, 1.59796433001491, 0.44687470279710395, 0.1859704277566732};
    private static final double[] WEIGHTS = configuredWeights();
    private static double[] configuredWeights() {
        String raw = System.getenv("R1_COMPONENT_RELATIVE_WEIGHTS");
        if (raw == null) raw = "1.5,1.25,1,1";
        String[] cells = raw.split(",", -1);
        if (cells.length != 4) throw new IllegalArgumentException("Four relative weights required");
        double[] out = new double[4]; double total = 0;
        for (int i=0;i<4;i++) {
            out[i] = Double.parseDouble(cells[i]);
            if (!Double.isFinite(out[i]) || out[i]<0) throw new IllegalArgumentException("Invalid task weight");
            total += out[i];
        }
        if (!Double.isFinite(total) || total<=0) throw new IllegalArgumentException("Positive finite weight sum required");
        for (int i=0;i<4;i++) out[i] /= total;
        return out;
    }
    public static double[] referenceScales() { return SCALES.clone(); }
    public static double[] authorWeights() { return WEIGHTS.clone(); }

    public static final class Measure {
        public final double value;
        public final Statistic statistic;
        public final String unit;
        public Measure(double value, Statistic statistic, String unit) {
            if (!Double.isFinite(value) || value < 0.0) throw new IllegalArgumentException("Measure must be finite and nonnegative");
            this.statistic = Objects.requireNonNull(statistic, "statistic");
            if (!statistic.unit.equals(unit)) throw new IllegalArgumentException("Statistic/unit mismatch: " + statistic);
            this.value = value;
            this.unit = unit;
        }
    }
    public static final class Evidence {
        public final String source, scope;
        public final EvidenceStatus status;
        public final DomainStatus domain;
        public final boolean stateClipped;
        public Evidence(String source, String scope, EvidenceStatus status, DomainStatus domain, boolean stateClipped) {
            this.source = required(source, "response_source"); this.scope = required(scope, "evidence_scope");
            this.status = Objects.requireNonNull(status, "evidence status"); this.domain = Objects.requireNonNull(domain, "domain status");
            this.stateClipped = stateClipped;
            if (stateClipped) throw new IllegalArgumentException("Numerically clipped motion state is not a supported response");
            if (status == EvidenceStatus.ADMITTED_WITHIN_SCOPE && "NONE".equalsIgnoreCase(scope))
                throw new IllegalArgumentException("Admitted declaration requires an external scope identifier");
        }
    }
    public static final class Result {
        private final double[] ratios, components;
        public final double rawTaskCost;
        public final String status;
        public final boolean overallSafetyAssessed = false, physicalValidationPerformed = false, probability = false;
        private Result(double[] ratios, double[] components, double cost, Mode mode) {
            this.ratios = ratios.clone(); this.components = components.clone(); this.rawTaskCost = cost;
            this.status = mode == Mode.DIAGNOSTIC ? "DIAGNOSTIC_NOT_ADMITTED" : "EXTERNAL_ADMISSION_DECLARED_TASK_COST_ONLY";
        }
        public double[] ratios() { return ratios.clone(); }
        public double[] components() { return components.clone(); }
    }
    public static Result score(Measure[] measures, Evidence evidence, Mode mode) {
        Objects.requireNonNull(evidence, "evidence"); Objects.requireNonNull(mode, "mode");
        if (measures == null || measures.length != 4) throw new IllegalArgumentException("Four centered SD channels are required");
        if (mode == Mode.EVIDENCE_GATED && (evidence.status != EvidenceStatus.ADMITTED_WITHIN_SCOPE || evidence.domain != DomainStatus.WITHIN_DECLARED_SCOPE))
            throw new IllegalArgumentException("Response has no externally declared admission within this input scope");
        double[] ratios = new double[4], components = new double[4]; double cost = 0.0;
        for (int i = 0; i < 4; i++) {
            Measure m = Objects.requireNonNull(measures[i], "channel " + i);
            if (m.statistic != EXPECTED[i] || !EXPECTED[i].unit.equals(m.unit))
                throw new IllegalArgumentException("Wrong statistic/channel order at " + i + "; expected " + EXPECTED[i]);
            ratios[i] = m.value / SCALES[i]; components[i] = WEIGHTS[i] * ratios[i]; cost += components[i];
            if (!Double.isFinite(ratios[i]) || !Double.isFinite(components[i]) || !Double.isFinite(cost))
                throw new IllegalArgumentException("Non-finite preference arithmetic");
        }
        return new Result(ratios, components, cost, mode);
    }
    private static String required(String value, String name) {
        if (value == null || value.trim().isEmpty()) throw new IllegalArgumentException(name + " is missing");
        return value;
    }
    private static final String[] INPUT_VALUES = {"roll_centered_sd_deg", "heading_centered_sd_deg", "sway_centered_sd_m_s", "surge_centered_sd_m_s"};
    private static final String[] PREFIXES = {"roll", "heading", "sway", "surge"};
    private static final String[] REQUIRED_HEADERS = {"id", "roll_centered_sd_deg", "heading_centered_sd_deg", "sway_centered_sd_m_s", "surge_centered_sd_m_s", "response_source", "evidence_status", "evidence_scope", "domain_status", "state_clipped"};
    private static final String[] OUTPUT_HEADERS = {"id", "policy_id", "mode", "status", "declared_evidence_status", "domain_status", "response_source", "evidence_scope", "roll_centered_sd_deg", "heading_centered_sd_deg", "sway_centered_sd_m_s", "surge_centered_sd_m_s", "roll_ratio", "heading_ratio", "sway_ratio", "surge_ratio", "roll_weighted", "heading_weighted", "sway_weighted", "surge_weighted", "raw_task_cost", "overall_safety_assessed", "physical_validation_performed", "probability", "preference_clipped", "static_means_policy"};
    private static List<String> parseCsv(String line) {
        List<String> fields = new ArrayList<>(); StringBuilder b = new StringBuilder(); boolean quoted = false, closed = false;
        for (int i = 0; i < line.length(); i++) {
            char c = line.charAt(i);
            if (quoted) {
                if (c == '"') { if (i + 1 < line.length() && line.charAt(i + 1) == '"') { b.append('"'); i++; } else { quoted = false; closed = true; } }
                else b.append(c);
            } else if (c == ',') { fields.add(b.toString()); b.setLength(0); closed = false; }
            else if (c == '"') { if (b.length() != 0 || closed) throw new IllegalArgumentException("Unexpected quote in CSV"); quoted = true; }
            else { if (closed) throw new IllegalArgumentException("Unexpected characters after CSV quote"); b.append(c); }
        }
        if (quoted) throw new IllegalArgumentException("Unclosed quote or multiline CSV cell");
        fields.add(b.toString()); return fields;
    }
    private static String csv(String value) {
        if (value.indexOf(',') >= 0 || value.indexOf('"') >= 0 || value.indexOf('\n') >= 0 || value.indexOf('\r') >= 0)
            return "\"" + value.replace("\"", "\"\"") + "\"";
        return value;
    }
    private static String join(List<String> row) {
        List<String> out = new ArrayList<>(); for (String s : row) out.add(csv(s)); return String.join(",", out);
    }
    private static String cell(List<String> row, Map<String, Integer> columns, String key) {
        return required(row.get(columns.get(key)), key);
    }
    private static boolean bool(String s) {
        if ("true".equals(s)) return true; if ("false".equals(s)) return false;
        throw new IllegalArgumentException("Boolean must be exactly true or false");
    }
    public static int runCsv(Path input, Path output, Mode mode) throws IOException {
        if (Files.exists(output)) throw new IllegalArgumentException("Output already exists; never overwrite results");
        List<String> lines = Files.readAllLines(input, StandardCharsets.UTF_8);
        if (lines.isEmpty()) throw new IllegalArgumentException("Input CSV is empty");
        String header = lines.get(0); if (header.startsWith("\ufeff")) header = header.substring(1);
        List<String> names = parseCsv(header); Map<String,Integer> columns = new LinkedHashMap<>();
        for (int i = 0; i < names.size(); i++) if (columns.put(names.get(i), i) != null) throw new IllegalArgumentException("Duplicate header " + names.get(i));
        for (String name : REQUIRED_HEADERS) if (!columns.containsKey(name)) throw new IllegalArgumentException("Missing header " + name);
        List<String> out = new ArrayList<>(); out.add(String.join(",", OUTPUT_HEADERS)); int count = 0;
        for (int line = 1; line < lines.size(); line++) {
            if (lines.get(line).isEmpty()) continue;
            try {
                List<String> row = parseCsv(lines.get(line)); if (row.size() != names.size()) throw new IllegalArgumentException("CSV field count differs from header");
                String id = cell(row, columns, "id"); Measure[] m = new Measure[4];
                for (int i = 0; i < 4; i++) {
                    Statistic st = EXPECTED[i]; String unit = st.unit;
                    if (columns.containsKey(PREFIXES[i] + "_statistic")) st = Statistic.valueOf(cell(row, columns, PREFIXES[i] + "_statistic"));
                    if (columns.containsKey(PREFIXES[i] + "_unit")) unit = cell(row, columns, PREFIXES[i] + "_unit");
                    m[i] = new Measure(Double.parseDouble(cell(row, columns, INPUT_VALUES[i])), st, unit);
                }
                Evidence ev = new Evidence(cell(row, columns, "response_source"), cell(row, columns, "evidence_scope"),
                    EvidenceStatus.valueOf(cell(row, columns, "evidence_status")), DomainStatus.valueOf(cell(row, columns, "domain_status")), bool(cell(row, columns, "state_clipped")));
                Result r = score(m, ev, mode); List<String> result = new ArrayList<>(Arrays.asList(id, POLICY_ID, mode.name(), r.status, ev.status.name(), ev.domain.name(), ev.source, ev.scope));
                for (Measure v : m) result.add(Double.toString(v.value));
                for (double v : r.ratios()) result.add(Double.toString(v));
                for (double v : r.components()) result.add(Double.toString(v));
                result.add(Double.toString(r.rawTaskCost));
                result.addAll(Arrays.asList("false", "false", "false", "false", "MEANS_UNKNOWN_UNLESS_INDEPENDENTLY_ESTABLISHED_NOT_ZERO_ASSUMED"));
                out.add(join(result)); count++;
            } catch (RuntimeException e) { throw new IllegalArgumentException("CSV physical line " + (line + 1) + ": " + e.getMessage(), e); }
        }
        if (count == 0) throw new IllegalArgumentException("Input has no data records");
        Files.write(output, out, StandardCharsets.UTF_8, StandardOpenOption.CREATE_NEW, StandardOpenOption.WRITE);
        return count;
    }
    public static void main(String[] args) {
        try {
            if (args.length == 1 && "--print-output-header".equals(args[0])) { System.out.println(String.join(",", OUTPUT_HEADERS)); return; }
            Map<String,String> options = new LinkedHashMap<>();
            for (int i = 0; i < args.length; i += 2) {
                if (i + 1 >= args.length || !Arrays.asList("--input", "--output", "--mode").contains(args[i])) throw new IllegalArgumentException("Use --input FILE --output NEW_FILE --mode DIAGNOSTIC|EVIDENCE_GATED");
                if (options.put(args[i], args[i + 1]) != null) throw new IllegalArgumentException("Duplicate option " + args[i]);
            }
            int count = runCsv(Path.of(required(options.get("--input"), "input")), Path.of(required(options.get("--output"), "output")), Mode.valueOf(required(options.get("--mode"), "mode")));
            System.out.println("rows=" + count + ";policy=" + POLICY_ID + ";physical_validation_performed=false;overall_safety_assessed=false");
        } catch (Exception e) { System.err.println("ERROR: " + e.getMessage()); System.exit(2); }
    }
}
