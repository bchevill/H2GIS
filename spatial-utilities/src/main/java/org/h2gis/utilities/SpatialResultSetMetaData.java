/*
 * h2spatial is a library that brings spatial support to the H2 Java database.
 *
 * h2spatial is distributed under GPL 3 license. It is produced by the "Atelier SIG"
 * team of the IRSTV Institute <http://www.irstv.fr/> CNRS FR 2488.
 *
 * Copyright (C) 2007-2012 IRSTV (FR CNRS 2488)
 *
 * h2patial is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * h2spatial is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * h2spatial. If not, see <http://www.gnu.org/licenses/>.
 *
 * For more information, please consult: <http://www.orbisgis.org/>
 * or contact directly:
 * info_at_ orbisgis.org
 */

package org.h2gis.utilities;

import java.sql.ResultSetMetaData;
import java.sql.SQLException;

/**
 * In order to provide a common API with H2 Spatial and PostGIS this MetaData give type information on Geometry fields.
 * @author Nicolas Fortin
 */
public interface SpatialResultSetMetaData extends ResultSetMetaData {

    /**
     * @param column
     * @return {@link GeometryTypeCodes} of the provided column.
     */
    int getGeometryType(int column) throws SQLException;

    /**
     * @return {@link GeometryTypeCodes} of the first geometry column.
     * @throws SQLException if this meta data does not contains a geometry field.
     */
    int getGeometryType() throws SQLException;

    /**
     * @return Column index of the first geometry in this result set.
     * @throws SQLException
     */
    public int getFirstGeometryFieldIndex() throws SQLException;
}
